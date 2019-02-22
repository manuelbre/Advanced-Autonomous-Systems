function [x] = pendulum(x, u)
    % constants
    A = 100.0; % rad/s^2
    B = 2.0; % rad/s
    C = 2.0; % rad/(s^2*V)
    x = -A * sin(x(1)) - B * x(2) + C * u;
end