function [x_G, y_G] = Laser2Global(x_L, y_L, X, d)
    %% Description
    % Conver points from Laser coordinate frame to global coordinate frame.
    %
    % Input:
    %       - x_L (Nx1): X-coordinate of points in laser coordinate frame
    %       - y_L (Nx1): Y-coordinate of points in laser coordinate frame
    %       - X (3x1): State [x, y, theta]
    %
    % Output:
    %       - x_G (Nx1): X-coordinate of points in global coordinate frame
    %       - y_G (Nx1): Y-coordinate of points in global coordinate frame

    
    %% Function
    
    % Flip X-axis
    x_1 = x_L * (-1);
    y_1 = y_L;

    % Shift Origin of frame to Center of kinematic model of robot
    x_2 = x_1;
    y_2 = y_1 + d;
    
    % Rotate and translate frame with respect to global coordinate frame
    G = X(1:2) + [cos(X(3) - pi/2) -sin(X(3) - pi/2); ...
                  sin(X(3) - pi/2)  cos(X(3) - pi/2)]* ...
                                                            [x_2.' ; y_2.'];
    x_G = G(1,:).';
    y_G = G(2,:).';
    %% Sanity check
    assert(isequal(size(x_G),size(y_G),size(x_L),size(y_L)))
end