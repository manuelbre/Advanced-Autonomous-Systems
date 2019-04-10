%% DESCRIPTON
% Main file for Assignment 2 of Advanced Autonomous System class at UNSW T1
% 2019
%
% This file loads IMU data and estimates position of the robot in the
% global coordinate frame. This is basic example and fully determinisitc at
% this point.

function main(path_data_dir, path_IMU_data, path_Speed_data, path_project1)
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

% Sanity checks
assert(isequal(t_raw_IMU, t_raw_v));

%% Part A) - Estimate yaw in global coordinate frame from IMU data.
theta = estimateAttitude(yaw_rate, t_w, attitude_set);
theta_plot = rad2deg(theta);

figure(1)
plot(t_w, theta_plot);
title('Integrated Yawrate.');
grid('on');
xlabel('t [s]');
ylabel('\theta_B [deg]');

%% Part B) - Estimate position in global frame
assert(isequal(t_v, t_w))
[X, Y] = estimatePosition(v, theta, t_v);

figure(2);
plot(X, Y);
hold on;
plot(X(1), Y(1), 'kx');
text(X(1), Y(1),' \leftarrow Start')
plot(X(end), Y(end), 'kx');
text(X(end), Y(end),' \leftarrow End')
title('Estimated Position of Robot.');
grid('on');
xlabel('x [m]');
ylabel('y [m]');

%% Part C) & D) - Plot OOI in global coordinate frame and Data Association

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
init_OOIs = plotLaserdataGlobalframe( laser_scans(:, i_laser), t_laser(i_laser), ...
                    X, OOI_fig.OOI_brilliant_init, OOI_fig.robot, OOI_fig.title,  ...
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
        X = updateState(X, t_vw(i_vw - 1), v(i_vw), w(i_vw), t_vw(i_vw));
        i_vw = i_vw + 1;
    end
    
    % More precise state estimation for difference in measurement frequency
    X_laser = updateState(X, t_vw(i_vw-1), v(i_vw), w(i_vw), ...
                                                        t_laser(i_laser));
    % Plot object of interests                                          
    OOIs = plotLaserdataGlobalframe( laser_scans(:, i_laser), t_laser(i_laser),...
                X_laser, OOI_fig.OOI_brilliant, OOI_fig.robot, OOI_fig.title, ...
                                                i_laser, laser_settings);
    % Data association
    OOIs = DataAssociationOOIs(OOIs, data_assoc_settings.dist_threshold, ...
                                                                init_OOIs);
    plotIDs(OOIs, OOI_fig.OOI_brilliant_txt, data_assoc_settings.extra_txt);
    
    % Increment laser data index
    i_laser = i_laser + laser_increment;
    pause(laser_t_pause) ;  % wait for ~10ms (approx.)
end

end
