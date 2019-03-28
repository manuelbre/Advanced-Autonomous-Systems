%% DESCRIPTON
% Main file for Assignment 2 of Advanced Autonomous System class at UNSW T1
% 2019
%
% This file loads IMU data and estimates position of the robot in the
% global coordinate frame. This is basic example and fully determinisitc at
% this point.

function main(path_data_dir, path_IMU_data, path_Speed_data)
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

% IMU Data parameters
% yaw rate channel
yawrate_channel = 6;
% time steps
t_step_w = 0.1e-3; % [s]

% Attitude estimation settings
attitude_set.t_stationary = 20; % [s]
attitude_set.inverted = true;
attitude_set.theta_start = pi/2; % [rad]

% Speed Data parameters
% time steps
t_step_v = 0.1e-3; % [s]

% Load files and specify data directory
if ~exist('path_data_dir','var'), path_data_dir = default_path_data_dir'; end
if ~exist('path_IMU_data','var'), path_IMU_data = default_path_IMU_data'; end
if ~exist('path_Speed_data','var'), path_Speed_data = ...
                                            default_path_Speed_data'; end
if ~exist('path_Laser_data','var'), path_Laser_data = ...
                                            default_path_Laser_data'; end

% Load IMU data from file and preprocess
load(path_IMU_data);
yaw_rate = IMU.DATAf(yawrate_channel,:); % [rad/s]
t_raw = double(IMU.times); % [-]W
t_w = (t_raw - t_raw(1))* t_step_w; % [s]

% Load Speed data from file and preprocess
load(path_Speed_data);
v = Vel.speeds; % [rad/s]
t_raw = double(Vel.times); % [-]W
t_v = (t_raw - t_raw(1))* t_step_v; % [s]

% Load Laser data from file and preprocess
dataL.comment

%% Estimate yaw in global coordinate frame from IMU data.
theta = estimateAttitude(yaw_rate, t_w, attitude_set);
theta_plot = rad2deg(theta);

figure(1)
plot(t_w, theta_plot);
title('Integrated Yawrate.');
grid('on');
xlabel('t [s]');
ylabel('\theta_B [deg]');

%% Estimate position in global frame
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

end
