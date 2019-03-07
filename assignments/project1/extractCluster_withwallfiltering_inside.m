function [Success, finished, Center, Size, p_cluster, next_idx] = extractCluster(X, Y)
    % Input:
    %       - X-Coordinate of object candidates
    %       - Y-Coordinate of object candidates
    %       -
    % Params
    threshold_n_range = 3; % Go to next cluster if those many points are not ...
                           % within range.
    threshold_wall = 0.05; % Threshold to discard points of a wall; [m] 
    r_range = [0.05 0.2]./ 2; % [m]
    finished = false;
 
    % Variables
    P = [X Y];
    d = []; % Distance
    N = 0; % Number of clusters
    idxs_consecutive_not_in_range = []; % Number of points not in range
    C_curr_cand = []; % Center candidates of current cluster
    C_next_cand = []; % Center candidate of next cluster
    p_curr_cluster = [];  % Points belonging to cluster candidate
    p_next_cluster = [];  % Points belonging to cluster candidate
    Center = []; % Center of objects
    Size = []; % Diameter of objects
    next_idx = []; % If threshold is met next cluster is evaluated
    p_cluster = []; % Points belonging to cluster
    Success = false; % True if Cluster was found

for i = 1:size(P,1)
        p = P(i, :);

        % Check 
        if isempty(C_curr_cand) && isempty(p_curr_cluster)
            C_curr_cand = p;
            p_curr_cluster = p;
            continue
        end
        
        % distance to cluster center
        r = pdist2(C_curr_cand, p);
        
        % Check if a the point of range is a wall
        if r > r_range(2) && ( ...
           pdist2(p_curr_cluster(end,:), p) < threshold_wall)
 
           idx_wall = i;
           while(pdist2(P(idx_wall,:), P(j,:) < threshold_wall) && ...
                 j <= size(P,1)
               idx_wall = j;
               j = j +1;
           end
           
           next_idx = j;
           Success = false;
           return
        
        else if r > r_range(2) && ...
                pdist2(p_curr_cluster(end,:), p) > threshold_wall)
            idxs_consecutive_not_in_range = ...
                    [idxs_consecutive_not_in_range i];
                
            % Check if this is consecutive
             if size(idxs_consecutive_not_in_range,2) >= 2 && ...
                idxs_consecutive_not_in_range(end-1) == i-1
                % Check if threshold is met
                if nnz(idxs_consecutive_not_in_range) > threshold_n_range
                    
                    % Set next start indx for next search
                    if ~ idxs_consecutive_not_in_range(end) == size(P,1)
                        next_idx = idxs_consecutive_not_in_range(1);
                    else
                        next_idx = [];
                    end
                    
                    % Check if current cluster satisfies cluster definition
                    dists = pdist2(C_curr_cand, p_curr_cluster);
                    if all(dists < r_range(2)) && ... % Check if all the points are within upper diameter range
                            max(dists) > r_range(1) % Check if the largest distance is larger than minimal diameter
                     
                     dists = pdist2(mean(), p_curr_cluster);
                     
                     % Found cluster. Add it to final set.
                     Success = true;
                     Center = C_curr_cand;
                     Size = max(dists) * 2;
                     p_cluster = p_curr_cluster;
                    end
                    
                   break
                end
             end
             
        else
            % Radius is within range or smaller than range.
            p_curr_cluster = [p_curr_cluster ; p];
            C_curr_cand = mean(p_curr_cluster, 1);
            idxs_consecutive_not_in_range = [];
        end
        end
end