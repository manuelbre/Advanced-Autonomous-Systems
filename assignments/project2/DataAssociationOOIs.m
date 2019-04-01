function [OOIs] = DataAssociationOOIs(OOIs, threshold, ref_OOI)
    %% Description
    % Associate brilliant OOI with reference brilliant OOIs.
    %
    % Input:
    %       - OOIs (struc): Object of Interested to be associated with an
    %                       ID.
    %       - threshold: Threshold distance to associate OOI.
    %       - ref_OOI: Reference object of interest.
    %
    % Output:
    %       - OOIs (struc): Updated Object of interest with associated ID.
    %
    
    %% Function
    
    if ~exist('threshold','var')
        threshold = 0.4;
    end
    
    OOIs.IDs = zeros(OOIs.N,1);
    
    if ~exist('ref_OOI','var')
        % Create initial OOIs IDs
        idx_init_brilliant = find(OOIs.Color ~= 0);
        OOIs.IDs(idx_init_brilliant) = [1: length(idx_init_brilliant)];
    else
        % Associate OOIs with reference OOIs
        
        idx_curr_brilliant_centers = find(OOIs.Color ~= 0);
        idx_ref_brilliant_centers = find(ref_OOI.Color ~= 0);
        
        % Distance bitween OOI pairs.
        D = pdist2(OOIs.Centers_G(OOIs.Color ~= 0,:), ...
                ref_OOI.Centers_G(ref_OOI.Color ~= 0,:));
        
        % Check what distance is minimum in column
        [mins_column, ~] = min(D, [], 1);
        idx_mins_column = D == mins_column;
        
        % Check what distance is below threshold.
        idx_below_threshold = D < threshold ;
        
        % Multiply pointwise to get 
        idx_min_and_below_threshold = find(idx_below_threshold.*idx_mins_column);
        [idx_OOIs, idx_ref_OOIs] = ind2sub(size(D), idx_min_and_below_threshold);
        % Copy ID from reference OOI.
        OOIs.IDs(idx_curr_brilliant_centers(idx_OOIs)) = ...
                    ref_OOI.IDs(idx_ref_brilliant_centers(idx_ref_OOIs));

    end
    
end