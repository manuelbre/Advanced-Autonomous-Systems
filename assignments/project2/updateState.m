
function [X] = updateState(X_prev, t_prev, v_cur, w_cur, t_cur)
    %% Descripton
    % Integrate measurement estimate position in global coordinate frame.
    % 
    % INPUT:
    %       - v_cur (1X1): Current Robot velocity measurement [m/s]
    %       - theta_cur (1X1): Robot Yaw in global coordinate Frame[rad]
    %       - t (1x1): Time at which to estimate new state [s]
    %       - X_prev (3x1) : Previous state 
    %
    % OUTPUT:
    %       - X (3x1): Updated State [x, y, theta]
    %       - t (1x1): Updated time [t]
        
    %%
    % Sanity Check
    assert(t_cur >= t_prev)
    
    % Init Variables
    X = zeros(3,1);
    
    %% Integrate measurement to update State
    dt = t_cur - t_prev; % [s]
    
    X(3) = X_prev(3) + w_cur * dt; % [rad]
    X(1) = X_prev(1) + v_cur * cos(X(3)) * dt; % [m]
    X(2) = X_prev(2) + v_cur * sin(X(3)) * dt; % [m]
        
end