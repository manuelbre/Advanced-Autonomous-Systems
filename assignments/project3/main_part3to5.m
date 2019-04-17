%% DESCRIPTON
% Main file for Assignment 3 part 3, 4 and 5 of Advanced Autonomous ...
% System class at UNSW T1 - 2019
%
% This file loads IMU data and estimates position of the robot in the
% global coordinate frame. This is basic example and fully determinisitc at
% this point.

function main(path_data_dir, path_IMU_data, path_Speed_data, ...
                                path_project1, default_path_project2)
%% Preprocessing
close all;
% Add that folder plus all subfolders to the path.
dirs = fileparts(which(mfilename)); 
addpath(genpath(dirs));

% Params 
% Default files and directories
default_path_data_dir = './data';
default_path_IMU_data = 'IMU_dataC.mat';
default_path_Speed_data = 'Speed_dataC.mat';
default_path_Laser_data = 'Laser__2C.mat';
default_path_project1 = '../project1/';
default_path_project2 = '../project2/';

% IMU Data parameters
% yaw rate channel
yawrate_channel = 6;
% time steps
t_step_w = 0.1e-3; % [s]

% Attitude estimation settings
attitude_set.t_stationary = 20; % [s]
attitude_set.inverted = true;
attitude_set.theta_start = pi/2; % [rad]

% Preprocess yawrate settings
yawrate_set.t_stationary = 20; % [s]
yawrate_set.inverted = true;

% Speed Data parameters
% time steps
t_step_v = 0.1e-3; % [s]

% Laser Data parameters
laser_increment = 1;
laser_freq = 100; % [Hz]
laser_t_pause = 1.0/laser_freq; % [s]
t_step_laser = 0.1e-3; % [s]

% Laserdata in global coordinate frame parameters
laser_settings.use_circle_fit = false;
laser_settings.verbose = false;
laser_settings.d = 0.46; % Offset of Laser [m]
global ABCD;
ABCD.flagPause=0;
X_0 = [0; 0 ; pi/2]; % [[m]; [m]; [rad]]

% Data association parameters
data_assoc_settings.dist_threshold = 0.4; % [m]
data_assoc_settings.extra_txt_init = "\leftarrow ";
data_assoc_settings.extra_txt = "       \leftarrow ";


% Load files and specify data directory
if ~exist('path_data_dir','var'), path_data_dir = default_path_data_dir; end
if ~exist('path_IMU_data','var'), path_IMU_data = default_path_IMU_data; end
if ~exist('path_Speed_data','var'), path_Speed_data = ...
                                            default_path_Speed_data; end
if ~exist('path_Laser_data','var'), path_Laser_data = ...
                                            default_path_Laser_data; end
if ~exist('path_project1','var'), path_project1 = ...
                                            default_path_project1; end
if ~exist('path_project2','var'), path_project2 = ...
                                            default_path_project2; end

% Load IMU data from file and preprocess
load(path_IMU_data);
yaw_rate = IMU.DATAf(yawrate_channel,:); % [rad/s]
t_raw_IMU = double(IMU.times) * t_step_w; % [s]
% Define a zero an absolute zero timepoint, such that other time lines can
% expressed with respect to it
t_0 =  t_raw_IMU(1) ; % [s]
t_w = t_raw_IMU - t_0; % [s]

% Load Speed data from file and preprocess
load(path_Speed_data);
v = Vel.speeds; % [rad/s]
t_raw_v = double(Vel.times) * t_step_v; % [s]
% Express time of speed measurements relative to IMU measurements
t_v = t_raw_v -t_0; % [s]

% Load Laser data from file
load(path_Laser_data);
t_raw_laser = double(dataL.times) * t_step_laser; % [s]
% Express time of laser data relative to IMU measurements
t_laser = t_raw_laser - t_0; % [s]
laser_scans = dataL.Scans ;

% Add path of project1
addpath(path_project1);

% Add path of project2
addpath(path_project2);

% Sanity checks
assert(isequal(t_raw_IMU, t_raw_v));

%% Variables EKF

% Statistic
stdev_yawrate = deg2rad(1.4); % [rad/second]
stdev_v = 0.4; % [m/second]
stdev_rangeMeasurement = 0.16; % [m]
stdev_angleMeasurement = deg2rad(0.16); % [rad]
bias_yawrate = deg2rad(2.0); % [rad/second]

% Initial covariance of state
P = zeros(size(X_0,1));

% Initial covariance of input
P_u = [stdev_v.^2 0;...
       0 stdev_yawrate.^2];
   
% Covariance of process model noise
Q_processModel =  diag( [ (0.01)^2 ,(0.01)^2 , (1*pi/180)^2]);

% Covariance of measurement Model
R = [stdev_rangeMeasurement.^2 0 ;...
     0 stdev_angleMeasurement.^2];

%% Plot OOI in global coordinate frame

% Create figure handle
OOI_fig = laserdata_figure(); 

% Preprocess yaw rate (Debiasing and change of coordinate system)
w = preprocessYawrate(yaw_rate, t_w, yawrate_set);

% Sanity checks
assert(isequal(t_raw_IMU, t_raw_v));

% Initialize
i_vw = 2;
i_laser = 1;
t_vw = t_w; % [s]
X = X_0; % [[m]; [m]; [rad]]

% Plot Initial OOI
init_OOIs = plotLaserdataGlobalframeProj3( laser_scans(:, i_laser), t_laser(i_laser), ...
                    X, OOI_fig.OOI_brilliant_init, OOI_fig.title,  ...
                                                i_laser, laser_settings);

% Data association
init_OOIs = DataAssociationOOIs(init_OOIs, ...
    data_assoc_settings.dist_threshold);
plotIDs(init_OOIs, OOI_fig.OOI_brilliant_init_txt, ...
    data_assoc_settings.extra_txt_init);

i_laser = i_laser + 1;

while 1
    if (ABCD.flagPause), pause(0.2) ; continue ; end
    if (i_laser>dataL.N) || (i_vw > length(t_vw)), break ;  end
        
    % Innerloop for higher frequency IMU and speed data
    while (t_vw(i_vw) <= t_laser(i_laser))
        
        % EKF Process Model
        dt = - t_vw(i_vw - 1) + t_vw(i_vw);
        assert(dt > 0);
        u =  [v(i_vw), w(i_vw)];
        J_X = J_X(X, u, dt);
        J_u = J_u(X, u, dt);
        P = J_X*P*J_X' + J_u*P_u*J_u.' + Q_processModel; % P(K+1|K)

        % Calculate expected value of X
        % X(k+1|k) = f( X(k|k), u(k) )
        X = processModel(X, u, t_vw(i_vw - 1), t_vw(i_vw));
        
        i_vw = i_vw + 1;
    end
    
        % More precise update due to diff in frequency of measruements
        % Calculate expected value of X
        % X(k+1|k) = f( X(k|k), u(k) )
        X_laser = processModel(X, u, t_vw(i_vw - 1), t_laser(i_laser));
        
        % EKF Measurement Model
        % Get Measurements
        [r_f, alpha_f, intens] = getMeasurements(laser_scans(:, i_laser));
        % Project Measurements to center of kinematic model
        [r, alpha] = projectMeasurements(r_f, alpha_f, laser_settings.d);
        
        % Extract object of interest and plot brillian OOI                   
        OOIs = plotLaserdataGlobalframeProj3( laser_scans(:, i_laser), t_laser(i_laser),...
                X_laser, OOI_fig.OOI_brilliant, OOI_fig.title, ...
                                                i_laser, laser_settings);
        
        % Data association
        OOIs = DataAssociationOOIs(OOIs, data_assoc_settings.dist_threshold, ...
                                                                    init_OOIs);
        plotIDs(OOIs, OOI_fig.OOI_brilliant_txt, data_assoc_settings.extra_txt);
        
         % Get brilliant OOI
        N_nonzero_IDs = nnz(OOIs.IDs);
        nonzero_IDs = OOIs.IDs(OOIs.IDs~=0);
        nonzero_C_G = OOIs.Centers_G(OOIs.IDs~=0,:);
        nonzero_angle_measurement = OOIs.angle_measurement(OOIs.IDs~=0,:);
        nonzero_range_measurement = OOIs.range_measurement(OOIs.IDs~=0,:);
        
        if N_nonzero_IDs > 0
            for ii = 1:N_nonzero_IDs
                ID = nonzero_IDs(ii);
                angle = nonzero_angle_measurement(ID);
                range = nonzero_range_measurement(ID);

                % Get Center of initial OOIs
                init_OOI_Center = init_OOIs.Centers_G(init_OOIs.ID == ID, :);

                % Measurement model to get expected measurements
                [E_range, E_angle, H] = measurementModel(init_OOI_Center, X);
                % Residual of actual measurements to expected measutements.
                z  = [ range - E_range               ; ...              
                       wrapToPi(angle - E_angle)];

                % Update state with help of residual and kalman Gain
                K = KalmanGain(R, H, P);
                X = X + K * z ;       % update the expected value of the state to X(k+1|k+1)
                P = P - K * H * P ;   % update the Covariance of the state P(k+1|k+1)
            end
        end
        
        % Robot Location
        set(OOI_fig.robot,'XData', X(1), 'YData', X(2), 'UData',...
                        cos(X(3)) ,'VData', sin(X(3)));

    
    % Increment laser data index
    i_laser = i_laser + laser_increment;
    pause(laser_t_pause) ;  % wait for ~10ms (approx.)
end

end

function [J] = J_X(X, u, dt)
    % Calculate Jacobian of process model with respect to X evaluated at X.

    if isequal(size(X), [3,1])
        J = [ 1, 0, -dt * u(1) * sin(X(3));....
              0, 1,  dt * u(1) * cos(X(3));...
              0, 0,  1                    ];

    elseif isequal(size(X), [4,1])
            
        J = [ 1, 0, -dt * u(1) * sin(X(3)),      0;...
              0, 1,  dt * u(1) * cos(X(3)),      0;...
              0, 0,  1                        , -dt;...
              0, 0,  0                        ,  1];
    else
        error('X has wrong dimension.');
    end
end

function [J] = J_u(X, u, dt)
    % Calculate Jacobian of process model with respect to u evaluated ...
    % at (X,u).

    if isequal(size(X), [3,1])
        J = [ dt * cos(X(3)) 0;...
              dt * sin(X(3)) 0;...
              0              dt];
          
    elseif isequal(size(X), [4,1])
            
        J = [ dt * cos(X(3)) 0;...
              dt * sin(X(3)) 0;...
              0              dt;
              0              0];
     
    else
        error('X has wrong dimension.');
    end
end

function [ranges, angles, intens] = getMeasurements(scan)
    %% Get Measurements of Lidar
    %
    % INPUT:
    %       - 'scan' : Scan data    
    % OUTPUT:
    %       - ranges (Nx1): Range measurements [m]
    %       - angles (Nx1): Angle measurements [rad]
    %       - intens (1xN): matrix containing intensity of reflectivity of
    %                       object.
   
    %% Function
    
    mask1FFF = uint16(2^13-1); % Buttom 13 bits
    maskE000 = bitshift(uint16(7),13); % Top 13 bits

    intens = bitand(scan,maskE000); 
    ranges = single(bitand(scan,mask1FFF))*0.01; % [m]   
    angles = [0:360]'*0.5* pi/180 ;  % [rad]
    u = [ranges.' angles.'];
    
end

function [ranges_proj, angles_proj] = projectMeasurements(ranges, angles, offset)
    %% Get Measurements of Lidar
    %  Modify laser measurements such that they where made on a virtual ...
    %  laser at the center of the kinematic model.
    %  This can be achieved by shifting the point by a vector v = [0 -d]...
    %  and then using that vector to determine the range and angle
    %
    % INPUT:
    %       - u (Nx2): Range [m] and angle measurements [rad
    %       - offset (1x1): Offset of laser in negative y distance in ...
    %                       robot coordinate frame.
    %                  
    % OUTPUT:
    %       - ranges_proj (Nx2): Range [m] projected  at center of ...
    %                            Kinematic Model.
    %       - angles_proj (Nx2): angles [rad] projected  at center of ...
    %                            Kinematic Model.
   
    %% Function
    vec = [ranges.* cos(angles) ranges.* sind(angles)];
    vec_offset = [0 offset];
    vec_projected = vec_offset + vec;
    ranges_proj = sqrt(vec_projected(:,1).^2 + vec_projected(:,2).^2);
    angles_proj = atan2(vec_projected(:,1),vec_projected(:,2));
    assert(isequal(size(ranges_proj),size(angles_proj)));
end

function [X] = processModel(X_prev, u_cur, t_prev, t_cur)
    %% Descripton
    % Integrate measurement estimate position in global coordinate frame.
    % 
    % INPUT:
    %       - X_prev (3x1) : Previous state [x, y, theta]
    %       - u_cur (2X1): Current input to robot, u_cur(1) = v [m/s] ...
    %                      and u_cur(2) = w [rad/s]
    %       - t_prev (1x1): Previous time of X_pre [s]
    %       - t_cur (1x1): Time at which to estimate the new state [s]
    %
    % OUTPUT:
    %       - X (3x1): Updated State [x, y, theta]
        
    %%
    % Sanity Check
    assert(t_cur >= t_prev)
    
    % Init Variables
    X = zeros(size(X_prev));
    
    %% Integrate measurement to update State
    dt = t_cur - t_prev; % [s]
    v = u_cur(1);
    w = u_cur(2);
    
    if length(X_prev) == 3
        X(3) = X_prev(3) + w * dt; % [rad]
        X(1) = X_prev(1) + v * cos(X(3)) * dt; % [m]
        X(2) = X_prev(2) + v * sin(X(3)) * dt; % [m]
    
    elseif length(X_prev) == 4
        X(3) = X_prev(3) + w * dt; % [rad]
        X(1) = X_prev(1) + v * cos(X(3)) * dt; % [m]
        X(2) = X_prev(2) + v * sin(X(3)) * dt; % [m]
    else
        error('X has wrong dimension')
    end
        
end
        
function [E_range, E_angle, H] = measurementModel(P_landmarks, X)
    %% measurementModel
    % Apply measurement model to current robot state to calculate ...
    % expected measurements.
    %
    % Input:
    %       - P_landmark (1x2): Position of Landmark in global ...
    %                           coordinate frame.
    %       - X (3x1): State of robot [x,y,theta]
    % Output:
    %       - E_range (1x1): Expected value of ranges. [m]
    %       - E_angle (1x1): Expected value of ranges. [rad]
    %       - H (2x4): Jacobian of Measurement function
    
    % Variables
    N = size(P_landmarks,1);
    
    dx = P_landmarks(:,1) - X(1);
    dy = P_landmarks(:,2) - X(2);
    E_range_sq = dx.^2 + dy.^2;
    E_range = sqrt(E_range_sq);
    E_angle = atan2(dy, dx) - X(3) + pi/2;
    
     % Jacobian of measuremnet model H=dh/du
    if length(X) == 4
        H = [ -dx./E_range,   -dy./E_range,     0, 0;...
               dy./E_range_sq, dy./E_range_sq, -1, 0];
    elseif length(X) == 3
        H = [ -dx./E_range,   -dy./E_range,     0;...
               dy./E_range_sq, dy./E_range_sq, -1];
    else
        error('X has wrong dimension')
    end
end

function [K] =  KalmanGain(R, H, P)
    %% KalmanGain
    % Calculate Kalman gain for extended kalman filter
    %
    % Input:
    %       - R: Covariance of Measurement model
    %       - H: Jacobian of measurement function with respect to input u.
    %       - P: Convariance of state X
    % Output:
    %       - K: Gain for Kalman filter
    %% Function
    S = R + H*P*H' ;
    K = P*H'*S^(-1) ;
end
