function OOI = ExtractOOIs(X, Y, intens)
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
    OOI.circle_MSE = [];
    OOI.p_c = {};
    X_ = X;
    Y_ = Y;
    finished = false;
    while ~finished
        [Success, finished, Centers, Diameters, p_c, intensities_c, next_idx] = ...
                                       extractCluster(X_, Y_, intens);
        
        
        if Success
            OOI.N = OOI.N + Success;
            OOI.Centers = [OOI.Centers; Centers];
            OOI.p_c(OOI.N) = {p_c};
            OOI.Diameters = [OOI.Diameters; Diameters];
            OOI.Color = [OOI.Color; any(intensities_c > 0)];
        end
        
        X_ = X_(next_idx:end, :);
        Y_ = Y_(next_idx:end, :);
        intens = intens(next_idx:end);
    end
    
    
    %%%% Cirlce fitting %%%%%
    
%     sizes_circle_fitting = [];
%     for i = 1:OOI.N
%         n = size(OOI.p_c{i},1);
%         par = CircleFitByPratt(OOI.p_c{i});
%         x_m = par(1);
%         y_m = par(2);
%         r_m = par(3);
%         sizes_circle_fitting = [sizes_circle_fitting;...
%             r_m*2];
%         circle_MSE = abs( (r_m).^2 - ((OOI.p_c{i}(:,1) - x_m) .^2 + ...
%             (OOI.p_c{i}(:,2) - y_m) .^2 ));
%         OOI.circle_MSE = [OOI.circle_MSE ; sum(circle_MSE)/n];
%         
%     end
%     
%     %   Within Radius
%     size_range = [0.05 0.2];
%     mask = sizes_circle_fitting < size_range(2) & ...
%         sizes_circle_fitting > size_range(1);

%     TODO: Uncomment next line
%     scatter(OOI.Centers(mask(:),1), OOI.Centers(mask(:),2),'ks')

end