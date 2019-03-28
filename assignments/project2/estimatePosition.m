
function [X, Y] = estimatePosition(v, theta, t)
    %% Descripton
    % Integrate velocity data to stimate position in global coordinate frame.
    % 
    % INPUT:
    %       - v (1XN): Robot Velocity [m/s]
    %       - theta (1XN): Robot Yaw in global coordinate Frame[rad]
    %       - t (1xN): Time [s]
    %
    % OUTPUT:
    %       - X (1xN): Positon along x-axis [m]
    %       - Y (1xN): Positon along y-axis [m]
    %
    
    %%
    % Params
    
    % Variables
    N = length(t);
    X = zeros(1,N);
    Y = zeros(1,N);
    %%
    
    % Integrate velocity
    for i = 2:N
        dt = t(i) - t(i-1);
        X(i) = X(i-1) + v(i-1) * cos(theta(i)) * dt;
        Y(i) = Y(i-1) + v(i-1) * sin(theta(i)) * dt;
    end
        
end