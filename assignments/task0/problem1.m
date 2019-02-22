%% Model of a pendulum
%% P1)c1)
% Params

% state
theta = pi/3.0;
dtheta = 0;
x_0 = [theta; dtheta];

% input
u = 0;

% time
t = 0;
dt = 0.001;
t_end = 7;

% helper
n_steps = t_end/dt;
x_history = zeros(2, n_steps);
x_history(1,1) = x_0(1);
x_history(2,1) = x_0(2);

% simulation
for i = 2:n_steps
    t = t + dt;
    x_history(1,i) = x_history(1,i-1) + dt.* x_history(2,i-1);
    x_history(2,i) = x_history(2,i-1) + dt.* pendulum(x_history(:,i-1), u);
end

figure()
plot(dt:dt:t_end, x_history)
title('Pendulum - Problem 1, c1')
xlabel('time [s]') 
legend({'\theta[rad]','d\theta [rad/s]'},'Location','southwest')

%% P1)c2)
% Params

% state
theta = pi/3.0;
dtheta = 0;
x_0 = [theta; dtheta];

% input
u = 10;

% time
t = 0;
dt = 0.001;
t_end = 7;

% helper
n_steps = t_end/dt;
x_history = zeros(2, n_steps);
x_history(1,1) = x_0(1);
x_history(2,1) = x_0(2);

% simulation
for i = 2:n_steps
    t = t + dt;
    x_history(1,i) = x_history(1,i-1) + dt.* x_history(2,i-1);
    x_history(2,i) = x_history(2,i-1) + dt.* pendulum(x_history(:,i-1), control_input_p1_c2(t));
end

figure()
plot(dt:dt:t_end, x_history)
title('Pendulum - Problem 1, c2')
xlabel('time [s]') 
legend({'\theta[rad]','d\theta [rad/s]'},'Location','southwest')

