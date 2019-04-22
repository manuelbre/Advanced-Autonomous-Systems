function [X_end, X_T] = fuzzyControl (map_range, d_virt_T_0, t_end, dt, disp_sim, vel_fis_file, ang_fis_file)
%% DESCRIPTION
%
% Simple Fuzzy control of a mobile robot. The goal is to move the robot ...
% from a start position S to a target positon T. This script is used to ...
% to demonstrate a simple, non-optimized version of fuzzy control.
%
% INPUT:
%       - map_range: Range of map in x and y direction [m]
%       - d_virt_T_0: Vortual target distance from real target [m]
%       - t_end: Simulation end time [s]
%       - dt: Timestep [s]
%       - disp_sim: Boolean if figure is to be shown while simulating.
%       - vel_fis_file:  Path to velocity fuzzin inference file.
%       - ang_fis_file: Path to angular fuzzin inference file.
%
% OUTPUT:
%       - X_end (3 x 1): Final robot state.
%       - X_T (3 x 1): Target state.


%% INITIALIZATION

% Preprocessing
close all;
% Add that folder plus all subfolders to the path.
dirs = fileparts(which(mfilename)); 
addpath(genpath(dirs));

% Params
default_vel_fis_file = 'MTRN4010_vel.fis';
default_ang_fis_file = 'MTRN4010_ang.fis';
if ~exist('vel_fis_file','var'), vel_fis_file = default_vel_fis_file; end
if ~exist('ang_fis_file','var'), ang_fis_file = default_ang_fis_file; end

% Size of Map
default_map_range= 50;
if ~exist('map_range','var'), map_range = default_map_range; end
map = [ -0.5 0.5;...
              -0.5 0.5].* map_range;
% Start location
S = [-20; -5];
% Start heading
% S_theta = -pi + 2 * pi * rand();
S_theta = 0;
% Target location
T = [10; 10];
% Target heading
% T_theta = -pi + 2 * pi * rand();
T_theta = 0;

X_T = [T; T_theta];

% Initial Velocity [m/s]
v_0 = 0;
% Initial turn rate [rad/s]
w_0 = 0;
% Time [s]
t = 0;
% End time [s]
t_end_default = 300;
if ~exist('t_end','var'), t_end = t_end_default; end
% Timestep [s]
dt_default = 0.1;
if ~exist('dt','var'), dt = dt_default; end

% Initial Virtual Target distance [m].
d_virt_T_0_default = 10;
if ~exist('d_virt_T_0','var'), d_virt_T_0 = d_virt_T_0_default; end
% Threshold to lower virtual target distance [m]
d_virt_T_threshold = 10; 
% Define update rule of virt target
virt_T_time_update = true;

% Load data
vel_fis = readfis(vel_fis_file);
ang_fis = readfis(ang_fis_file);

% Disp figure flag
disp_sim_default = true;
if ~exist('disp_sim','var'), disp_sim = disp_sim_default; end



%% FUNCTIONALITY
X = [S; S_theta]; % Robot state [x; y; theta]
u = [v_0; w_0]; % Input
d_virt_T = d_virt_T_0; % Virtual target distance [m]
X_virt = get_virt_target(X_T, d_virt_T_0); % Virtual Target state [m[

% Create figure
if disp_sim
    h = create_figure([map(1, :) map(2, :)], X);
end

while t <= t_end
    % Distance to virtual target
    d_dist = pdist([X(1:2).'; X_virt(1:2).']);
    % Angular difference to virtual target
    phi = atan2(X_virt(2) - X(2), X_virt(1) - X(1));
    d_ang = wrapToPi(phi - X(3));
    
    % Update virtual target distance
    if virt_T_time_update
        % Update virtual target distance according to time
        d_virt_T = d_virt_T_0 * (t_end - t) ./t_end;
    else
        % Update virtual target distance with respect to current distance
        if d_dist <= d_virt_T_threshold
            d_virt_T = d_virt_T_0 * d_dist/d_virt_T_threshold;
        end
    end
    
    % Update virtual target state
    X_virt = get_virt_target(X_T, d_virt_T);

    % Evaluate fuzzy control inference to get control inputs
    u(1) = evalfis(d_dist, vel_fis);
    u(2) = evalfis(d_ang, ang_fis);

    % Update robot state
    X = kinematicModel(X, u, dt);
    
    % Update figure
    if disp_sim
        update_figure(h, X, X_virt, X_T, d_dist, d_ang, t)
    end
    
    % Timestep
    t = t + dt;
end

X_end = X;

end

function [X_virt] = get_virt_target(X_T, d_virt_T)
%% DESCRIPTION
%
% Calculate state of virtual target according to distance of virtual ...
% target to real target.
%
% INPUT:
%       - X_T (3 x 1): Target state [x, y, theta]
%       - d_virt_T (1 x 1): Distance of virtual target to real target
%
% OUTPUT:
%       - X_virt (3 x 1): Virtual target state [x, y, theta]
%

%% FUNCTIONALITY

% Init
X_virt = zeros(3,1);

% Heading stays the same
X_virt(3) = X_T(3);

% Calculate X and Y coordinate
X_virt(1) = X_T(1) - d_virt_T * cos(X_T(3));
X_virt(2) = X_T(2) - d_virt_T * sin(X_T(3));

end

function [X_new] = kinematicModel(X_curr, u, dt)
%% DESCRIPTON
% Kinematic model of wheeled robot.
%
% INPUT:
%       - X_T (3 x 1): Current robot state [x, y, theta]
%       - u (2 x 1): Control input [v, w]
%       - dt (1 x 1): Timestep [s]
%
% OUTPUT:
%       - X_new (3 x 1): Robot state at next timestap [x, y, theta]
%
%% FUNCTIONALITY

X_new = X_curr + dt.* [u(1) * cos(X_curr(3)); ...
                       u(1) * sin(X_curr(3)); ...
                       u(2)                 ];
end

function [h] = create_figure(axis_range, X_0)
%% Create figure
% INPUT: 
%       - axis_range (1 x 4): Range of axis to be used for figure.
%       - X_0 (3 x 1): Initial robot position [x ; y ; theta]
%
% OUTPUT:
%       - h: Struc containing figure handles.
%

%% FUNCTIONALLITY

% Shape of robot to show orientation and position
h.robot_shape = [ 2 0; 1 1; -1 1; -1 -1; 1 -1; 2 0]';
% Rotation function
h.rot = @(X) [  cos(X(3)) -sin(X(3)); ...
                    sin(X(3))  cos(X(3))];
% Update shape function
h.get_shape = @(X) h.rot(X) * h.robot_shape + repmat([X(1); X(2)], 1, 6);

h.fig = figure('units', 'normalized', 'position', [0.1 0.2 0.5 0.5]);
axis(axis_range);
hold on;
grid on;
axis equal;
xlabel('X [m]');
ylabel('Y [m]');

% Create the plots
h.trace = plot(X_0(1), X_0(2), 'color', [0 0.66 0]);
h.robot = plot(h.robot_shape(1,:), h.robot_shape(1,:), 'color', 'b', 'linewidth', 2);
h.virt_T = plot(h.robot_shape(1,:), h.robot_shape(1,:), 'color', 'm', 'linewidth', 2);
h.T = plot(h.robot_shape(1,:), h.robot_shape(1,:), 'color', 'g', 'linewidth', 2);

hold off;
end

function [] = update_figure(h, X, X_virt, X_T, d_dist, d_ang, t)
%% Update and show figure
%
% INPUT: 
%       - h: Struc containing figure handles
%       - X (3 x 1): Robot state [x ; y ; theta]
%       - X_virt (3 x 1): Vortual state [x ; y ; theta]
%       - X_T (3 x 1): Target state [x ; y ; theta]
%       - t (1 x 1): Time [s]
%
% OUTPUT:
%       - None
%

%% FUNCTIONALLITY

% Get shapes
shape_X = h.get_shape(X);
shape_X_virt = h.get_shape(X_virt);
shape_X_T = h.get_shape(X_T);

% Update data
set(h.trace, 'xdata', [h.trace.XData X(1)], 'ydata', [h.trace.YData X(2)]);
set(h.robot, 'xdata', shape_X(1,:) ,'ydata', shape_X(2,:)); 
set(h.virt_T, 'xdata', shape_X_virt(1,:) ,'ydata', shape_X_virt(2,:)); 
set(h.T, 'xdata', shape_X_T(1,:) ,'ydata', shape_X_T(2,:)); 

% Update title and pause shortly
title(sprintf('Time %0.3f, d_{dist} = %0.3f m, d_{ang} = %0.3f rad', t, d_dist, d_ang));
pause(0.001);
end