%% Discrete Time Model
%% P2
% Params

A = [ 0.9 0.1 0.1 ;...
      0.1 0.9 0.3 ;...
      0.0 -0.3 0.9];
B = [0.0 0.0 1.0].' ;

k = 0:500;

% state
x_history = zeros(3,length(k));

%% P2 a)

x_0 = [0.0 0.0 1.0].' ;
x_history(:,1) = x_0;

for i = k+1
    x_history(:,i+1) = A * x_history(:, i)  + B * control_input_p1_c2(i-1);
end

figure()
plot(1:length(x_history), x_history)
title('Problem 2, a)')
xlabel('k [-]') 
legend({'x_1','x_2', 'x_3'},'Location','southwest')

figure()
title('Problem 2, a)')
plot3(x_history(1,:), x_history(2,:), x_history(3,:))

%% P2 b)

x_0 = [10.0 20.0 15.0].' ;
u = 0;
x_history(:,1) = x_0;


for i = k+1
    x_history(:,i+1) = A * x_history(:, i)  + B * u;
end

figure()
plot(1:length(x_history), x_history)
title('Problem 2, a)')
xlabel('k [-]') 
legend({'x_1','x_2', 'x_3'},'Location','southwest')

figure()
title('Problem 2, a)')
plot3(x_history(1,:), x_history(2,:), x_history(3,:))
