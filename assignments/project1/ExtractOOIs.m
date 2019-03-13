function OOI = ExtractOOIs(X, Y, intens, use_circle_fit)
    % Extract Object of interest out of laser scan data
    %
    % INPUT:
    %       - X: (1xN) matrix containing X Coordinates of laserdata 
    %       - Y: (1xN) matrix containing Y Coordinates of laserdata 
    %       - intens: (1xN) matrix containing intensity of reflectivity of
    %                 object.
    %
    % OUTPUT:
    %       - OOI: Struct of Objects of Interests with an apparent
    %              Diameter, Center and Color.

    OOI.N = 0; 
    OOI.Centers = [];
    OOI.Diameters   = [];
    OOI.Color = [];
    OOI.p_c = {};
    X_ = X;
    Y_ = Y;
    finished = false;
    
    while ~finished
        [Success, finished, Centers, Diameters, p_c, intensities_c, next_idx] = ...
                                       extractCluster(X_, Y_, intens, use_circle_fit);
        
        
        if Success
            assert(Success);
            assert(isequal(size(Centers), [1, 2]))
            assert(isequal(size(Diameters), [1, 1]))
            assert(Diameters <= 0.2 && Diameters >= 0.05)
            
            OOI.N = OOI.N + Success;
            OOI.Centers = [OOI.Centers; Centers];
            OOI.p_c(OOI.N) = {p_c};
            OOI.Diameters = [OOI.Diameters; Diameters];
            OOI.Color = [OOI.Color; any(intensities_c > 0)];
            assert(size(OOI.Centers,1) == OOI.N)
            assert(size(OOI.Diameters,1) == OOI.N)
            assert(isequal(size(OOI.Color), [OOI.N, 1]))
            assert(OOI.Color(end) == 0 || OOI.Color(end) == 1)

        end
        
        X_ = X_(next_idx:end, :);
        Y_ = Y_(next_idx:end, :);
        intens = intens(next_idx:end);
    end

end