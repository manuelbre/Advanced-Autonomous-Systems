
function [yaw_rate] = preprocessYawrate(yaw_rate, t, opt)
    %% Decsription
    % Debiasing and change of coordinate frame for gyroscope yaw rate.
    % 
    % INPUT:
    %       - yaw_rate (1XN): Yaw rate of robot [rad/s]
    %       - t (1xN): Time [s]
    %       - options (struct): Settings for
    %
    % OUTPUT:
    %       - theta (1xN): Integrated yaw [rad]
    %
    
    %% Init
    % Params
    
    % Settings
    if ~exist('opt','var')
        opt.t_stationary = 0; % [s]
        opt.inverted = false;
        opt.theta_start = 0; % [rad]
    end
    
    % Variables
    N = length(t);
    idx_statioary = find(t<=opt.t_stationary, 1, 'last');
    opt.theta_stationary = zeros(1,idx_statioary);
    %% Logic
    
    % Depending on convention on coordinate system need to invert frame
    yaw_rate = yaw_rate*(-1)^(opt.inverted);
    
    % Estimate bias while vehicle is stationary
    yaw_rate_bias = mean(yaw_rate(1:idx_statioary));
    
    % Debias
    yaw_rate = yaw_rate - yaw_rate_bias;
        
%     theta = wrapTo2Pi(theta);
end