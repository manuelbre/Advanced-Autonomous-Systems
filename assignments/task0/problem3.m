%% Wheeled platform
%% P3)c)
% Params

% state
x_0 = zeros(3,1);

% Input
u = zeros(2,1);

% Const
L = 2;

% time
t = 0;
dt = 0.01;
t_end = 2;

% helper
n_steps = t_end/dt;
x_history = zeros(3, n_steps);
x_history(:,1) = x_0;


%% c1) Constant angle
u(1) = 3;
u(2) = pi/6;


% simulation
for i = 2:n_steps
    t = t + dt;
    x_history(1,i) = x_history(1,i-1) + dt.* u(1) * cos(x_history(3,i-1));
    x_history(2,i) = x_history(2,i-1) + dt.* u(1) * sin(x_history(3,i-1));
    x_history(3,i) = x_history(3,i-1) + dt.* tan(u(2)) * u(1)/L;
end

figure()
title('Wheeled platform - Constant steering')
plot(x_history(1,:), x_history(2,:))
xlabel('x [m]') 
ylabel('y [m]')

%% c2) Eight shape
u(1) = 3;
u(2) = pi/3;
n_steps = 2000;


% simulation
for i = 2:n_steps
    if i >= n_steps/2
        u(2) = -pi/3;
    end
    t = t + dt;
    x_history(1,i) = x_history(1,i-1) + dt.* u(1) * cos(x_history(3,i-1));
    x_history(2,i) = x_history(2,i-1) + dt.* u(1) * sin(x_history(3,i-1));
    x_history(3,i) = x_history(3,i-1) + dt.* tan(u(2)) * u(1)/L;
end

figure()
title('Wheeled platform - Constant steering')
plot(x_history(1,:), x_history(2,:))
xlabel('x [m]') 
ylabel('y [m]')

