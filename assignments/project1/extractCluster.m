function [success, finished, Center, Diameter, p_cluster, intensities_c, next_idx] = ...
        extractCluster(X, Y, intensities, use_circle_fit)
    %% extractCluster
    % Extract cluster of data points according to defenition in exercise
    % sheet. This function is designed to be used recursively.
    %
    % INPUT:
    %       - X: (N x 1) matrix containing X Coordinates of laserdata 
    %       - Y: (N x 1) matrix containing Y Coordinates of laserdata 
    %       - intens: (1xN) matrix containing intensity of reflectivity of
    %                 object.
    %
    % OUTPUT:
    %       - success (boolean): True if cluster according to defenition was found.
    %       - finished (boolean): True if no points are left in point cloud.
    %       - Center (1 x 2): Center of found cluster.
    %       - Diameter (1 x 2): Diamater of found cluster.
    %       - p_cluster (N x 2): Points belonging to found cluster.
    %       - intensities_c (N x 1): Inensities of points of found cluster.
    %       - next_idx (1 x 1): Intex of point of next identified cluster
    %       candidate. Used for next iteration loop.
    %       - OOI: Struct of Objects of Interests with an apparent
    %              Diameter, Center and Color.
    
    %% Params
    threshold_n_range = 3; % Go to next cluster if those many points are not ...
                           % within range.
    threshold_neighbourhood = 0.05; % Threshold to discard points of a wall; [m] 
    threshold_neighbourhood_squared = threshold_neighbourhood.^2;
    r_range = [0.05 0.2]./ 2; % [m]
    r_range_squared = r_range.^2;
    threshold_n_points = 3;
 
    %% Variables
    P = [X Y];
    d = []; % Distance
    N = 0; % Number of clusters
    idxs_consecutive_not_in_range = []; % Number of points not in range
    C_curr_cand = []; % Center candidates of current cluster
    p_cluster = [];  % Points belonging to cluster candidate
    intensities_c = []; % Intensities of the cluster points
    Center = []; % Center of objects
    Diameter = []; % Diameter of objects
    next_idx = []; % If threshold is met next cluster is evaluated
    success = false; % True if Cluster was found
    finished = false;
    
    %% Logic
    
for i = 1:size(P,1)
        p = P(i, :);

        % Populate matrix for first round.
        if isempty(C_curr_cand) && isempty(p_cluster)
            C_curr_cand = p;
            p_cluster = p;
            intensities_c = intensities(i);
            continue
        end
        
        % distance to cluster center
        if use_circle_fit
            [C_curr_cand, r_squared] = CircleFit([p_cluster; p]);
        else
            C_curr_cand = sum(p_cluster, 1)/size(p_cluster,1);
            r_squared = sum((C_curr_cand - p).^2);
        end     
        
        dist_neighbour_squared = sum((p_cluster(end,:) - p).^2);
        
        % Check check the current radius and if the point is close to last point
        if r_squared > r_range_squared(2) &&  ...
            dist_neighbour_squared > threshold_neighbourhood_squared
        
            idxs_consecutive_not_in_range = ...
                    [idxs_consecutive_not_in_range i];
                
            % Check if this is consecutive therefore current cluster is
            % finished.
             if size(idxs_consecutive_not_in_range,2) >= 2 && ...
                idxs_consecutive_not_in_range(end-1) == i-1
            
                % Check if threshold is met
                if nnz(idxs_consecutive_not_in_range) > threshold_n_range
                    
                    % Set start index for next search
                    next_idx = idxs_consecutive_not_in_range(1);
                    
                    % Check if current cluster satisfies cluster definition
                    if size(p_cluster,1) >= threshold_n_points % Check if enough points are part of cluster
                        
                        % Get distance of all points to Cluster center
                        dists = sum((Center - p_cluster).^2, 2);
                        if all(dists < r_range_squared(2)) && ... % Check if all the points are within upper diameter range
                                max(dists) > r_range_squared(1) % Check if the largest distance is larger than minimal diameter

                         % Found cluster. Add it to final set.
                         success = true;
                         if ~use_circle_fit
                            Diameter = sqrt(max(dists)) * 2;
                         end
                            
                        else
                            % Points are not in the threshold therefore it is
                            % does not satisfy OOI definition
                            [success, Center, Diameter, p_cluster, ...
                                intensities_c] = discardCluster();
                        end
                    else
                        % Points are not in the threshold therefore it is
                        % does not satisfy OOI definition
                            [success, Center, Diameter, p_cluster, ...
                                intensities_c] = discardCluster();

                    end
                    return
                end
             end
             
        else
            % Radius is within range or smaller than range or point is
            % neighbor of last point.
            p_cluster = [p_cluster ; p];
            intensities_c = [intensities_c; intensities(i)];
            idxs_consecutive_not_in_range = [];
            Center = C_curr_cand;
            if use_circle_fit
                Diameter = sqrt(r_squared) * 2;                
            end
        end
end

% Edge case if all points were treated
finished = true;
% Check if points are OOI
dists = sum((C_curr_cand - p_cluster).^2, 2);
if all(dists < r_range_squared(2)) && ... % Check if all the points are within upper diameter range
    max(dists) > r_range_squared(1) && ...% Check if the largest distance is larger than minimal diameter
    size(p_cluster,1) >= threshold_n_points % Check if enough points are part of cluster

     % Found cluster. Add it to final set.
     success = true;
     Diameter = sqrt(r_squared) * 2;
else
    % Points are not in the threshold therefore it is
    % does not satisfy OOI definition
    [success, Center, Diameter, p_cluster, intensities_c] = discardCluster();
end

end

function [success, Center, Diameter, p_cluster, intensities_c] = discardCluster()
    success = false;
    Center = [];
    Diameter = 0;
    p_cluster = [];
    intensities_c = [];
end