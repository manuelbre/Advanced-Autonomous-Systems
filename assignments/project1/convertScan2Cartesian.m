function [X, Y, intensities] = convertScan2Cartesian(scan)
%% convertScan2Cartesian
% Decode scan data and convert to cartesian coordinates.
%
% INPUT: 
%       -scan: Array of class uint16, which encodes range and intensity 
%
% OUTPUT:
%       - X: (N x 1) matrix containing X Coordinates of laserdata 
%       - Y: (N x 1) matrix containing Y Coordinates of laserdata 
%       - intens: (1xN) matrix containing intensity of reflectivity of
%                 object.
    
%%
% Extract range and intensity, here.
% useful masks, for dealing with bits.
mask1FFF = uint16(2^13-1);
maskE000 = bitshift(uint16(7),13)  ;

intensities = bitand(scan,maskE000);  
ranges    = single(bitand(scan,mask1FFF))*0.01; % [m]
% Ranges expressed in meters, and unsing floating point format (e.g. single).

% 2D points, expressed in Cartesian. From the sensor's perpective.
angles = [0:360]'*0.5* pi/180 ;  % associated angle, for each individual range in a scan
X = cos(angles).*ranges; % [m]
Y = sin(angles).*ranges; % [m]    

end
