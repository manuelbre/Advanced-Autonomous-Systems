
function [theta] = estimateAttitude(yaw_rate, t, opt)
    %%
    % Integrate Gyroscope yaw_rate from robot to estimate yaw in global ...
    % coordinate frame.
    % 
    % INPUT:
    %       - yaw_rate (1XN): Yaw rate of robot [rad/s]
    %       - t (1xN): Time [s]
    %
    % OUTPUT:
    %       - theta (1xN): Integrated yaw [rad]
    %
    
    %%
    % Params
    
    % Settings
    if ~exist('opt','var')
        opt.t_stationary = 0; % [s]
        opt.inverted = false;
        opt.theta_start = 0; % [rad]
    end
    
    % Variables
    N = length(t);
    theta = zeros(1,N) + opt.theta_start;
    idx_statioary = find(t<=opt.t_stationary, 1, 'last');
    opt.theta_stationary = zeros(1,idx_statioary);
    %%
    
    % Depending on convention on coordinate system need to invert frame
    yaw_rate = yaw_rate*(-1)^(opt.inverted);
    
    % Estimate bias while vehicle is stationary
    yaw_rate_bias = mean(yaw_rate(1:idx_statioary));
    
    % Integrate angular rate
    for i = 2:N
        dt = t(i) - t(i-1);
        theta(i) = (theta(i-1)  + (yaw_rate(i) - yaw_rate_bias)*dt );
    end
        
%     theta = wrapTo2Pi(theta);
end