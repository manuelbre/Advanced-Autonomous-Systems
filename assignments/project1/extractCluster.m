function [Success, finished, Center, Size, p_cluster, intensities_c, next_idx] = ...
        extractCluster(X, Y, intensities)
    % Input:
    %       - X-Coordinate of object candidates
    %       - Y-Coordinate of object candidates
    %       -
    % Params
    threshold_n_range = 3; % Go to next cluster if those many points are not ...
                           % within range.
    threshold_neighbourhood = 0.05; % Threshold to discard points of a wall; [m] 
    threshold_neighbourhood_squared = threshold_neighbourhood.^2;
    r_range = [0.05 0.2]./ 2; % [m]
    r_range_squared = r_range.^2;
    threshold_n_points = 3;
 
    % Variables
    P = [X Y];
    d = []; % Distance
    N = 0; % Number of clusters
    idxs_consecutive_not_in_range = []; % Number of points not in range
    C_curr_cand = []; % Center candidates of current cluster
    p_cluster = [];  % Points belonging to cluster candidate
    intensities_c = []; % Intensities of the cluster points
    Center = []; % Center of objects
    Size = []; % Diameter of objects
    next_idx = []; % If threshold is met next cluster is evaluated
    Success = false; % True if Cluster was found
    finished = false;

for i = 1:size(P,1)
        p = P(i, :);

        % Check 
        if isempty(C_curr_cand) && isempty(p_cluster)
            C_curr_cand = p;
            p_cluster = p;
            intensities_c = intensities(i);
            continue
        end
        
        % distance to cluster center
        r_squared = sum((C_curr_cand - p).^2);
        dist_neighbour_squared = sum((p_cluster(end,:) - p).^2);
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
                    
                    % Set next start indx for next search
                    next_idx = idxs_consecutive_not_in_range(1);
                    
                    % Check if current cluster satisfies cluster definition
                    if size(p_cluster,1) >= threshold_n_points % Check if enough points are part of cluster
                        
                        dists = sum((C_curr_cand - p_cluster).^2, 2);
                        if all(dists < r_range_squared(2)) && ... % Check if all the points are within upper diameter range
                                max(dists) > r_range_squared(1) && ...% Check if the largest distance is larger than minimal diameter
                                size(p_cluster,1) >= threshold_n_points 

                         % Found cluster. Add it to final set.
                         Success = true;
                         Center = C_curr_cand;
                         Size = sqrt(max(dists)) * 2;
                        else
                            % Points are not in the threshold therefore it is
                            % does not satisfy OOI definition
                            Success = false;
                            Center = [];
                            Size = 0;
                            p_cluster = [];
                            intensities_c = [];
                        end
                    else
                        % Points are not in the threshold therefore it is
                        % does not satisfy OOI definition
                        Success = false;
                        Center = [];
                        Size = 0;
                        p_cluster = [];
                        intensities_c = [];
                    end

                   return
                end
             end
             
        else
            % Radius is within range or smaller than range or point is
            % neighbor of last point.
            p_cluster = [p_cluster ; p];
            intensities_c = [intensities_c; intensities(i)];
            C_curr_cand = sum(p_cluster, 1)/size(p_cluster,1);
            idxs_consecutive_not_in_range = [];
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
     Success = true;
     Center = C_curr_cand;
     Size = sqrt(max(dists)) * 2;
else
    % Points are not in the threshold therefore it is
    % does not satisfy OOI definition
    Success = false;
    Center = [];
    Size = 0;
    p_cluster = [];
    intensities_c = [];
end

end