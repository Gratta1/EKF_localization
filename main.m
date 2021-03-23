%% 2D EKF for localization of mobile robot and known marker association
%% Robot motion model
% state vector X = [x, y, theta]
% input u = [v, w] v: linear velocity; w: angular velocity
% dt = delta of time between inputs
% Model of a unicycle with wheel radius = 1
clc
clear

% Create random input for the robot
vmin= 1;
vmax= 3;
v=vmin+rand()*(vmax-vmin);
wmin= -3;
wmax= 3;
w=wmin+rand()*(wmax-wmin);

% Create noise and state of the robot
Q = [ 0.0005, 0, 0; 0, 0.0005, 0; 0, 0, 0.00005 ];
N = [ 0.002, 0; 0, 0.0002 ];
noise = mvnrnd([0,0,0], Q)';
landmark = [10, 30; -10, -26; 78, -45; -68, 14 ];
% n_landmarks = size(landmark, 1);
X = [0; 0; 0];
X_true = X + noise;
dt = 0.01;

trajectory_x = X(1);
trajectory_y = X(2);
trajectory_x_true = X_true(1);
trajectory_y_true = X_true(2);
theta = X(3);
theta_true = X_true(3);
update = [];
covariance_x = [];
covariance_y = [];
covariance_theta= [];
i = 0;
T = 0;
% rng('default')

diverge = 0;

covariance = Q;
bearing_array = [];
bearing_array_true = [];
command_array = [];
model_noise_array = noise;
%% Loop
while( T < 10000 && diverge == 0)

% Create random input
v=vmin+rand()*(vmax-vmin);
w=wmin+rand()*(wmax-wmin);
u = [v, w];

% Create state and measure noise
noise = mvnrnd([0,0,0], Q)';
noise_measure = mvnrnd([0, 0], N)';
model_noise_array = [model_noise_array; noise];
% Move forward the robot (adding noise) and prediction step
covariance = predict_covariance(X, covariance, Q, u, dt);
X_true = move_forward(X_true, u, dt, noise);
X = prediction_step(X, u, dt);

% Measure the distance from the landmark and make the measurement step for
% EKF
[range_true, bearing_true] = measure_step(X_true, landmark, noise_measure);
[range, bearing] = measure_prediction(X, landmark);

% if (abs(bearing(1)-bearing_true(1)) > 0.5 || abs(bearing(2)-bearing(2)) > 0.5 || abs(bearing(3)-bearing_true(3)) > 0.5 || abs(bearing(4)-bearing(4)) > 0.5)
%     diverge = 1;
%     X_prior = X;
% end

update(1) = 0; 
update(2) = 0;

if mod(i, 1) == 0 && i~=0
[X, covariance] = update_step(X, landmark, range, bearing, range_true, bearing_true, covariance, N);
update(i,1) = X(1);
update(i,2) = X(2);
end

% Update trajectories for plot
trajectory_x = [trajectory_x, X(1)];
trajectory_y = [trajectory_y, X(2)];
trajectory_x_true = [trajectory_x_true, X_true(1)];
trajectory_y_true = [trajectory_y_true, X_true(2)];
theta = [theta, X(3)];
theta_true = [theta_true, X_true(3)];
covariance_x = [covariance_x, covariance(1,1)];
covariance_y = [covariance_y, covariance(2,2)];
covariance_theta= [covariance_theta, covariance(3,3)];
bearing_array_true = [bearing_array_true; bearing_true];
bearing_array = [bearing_array; bearing];
command_array = [command_array; u];
i = i+1;
T = T+dt;


end

%% Plot trajectories and landmark position
figure  
hold on 
plot(trajectory_x_true, trajectory_y_true)
% for j=1:4
% plot(landmark(j,1), landmark(j,2), 'x')
% end
plot(trajectory_x, trajectory_y)
% plot(update(:,1), update(:,2), 'x')
axis equal
% 
%% Plot other stuff
figure  
subplot(2,1,1)
plot(trajectory_x)
subplot(2,1,2)
plot(trajectory_x_true)

figure  
subplot(2,1,1)
plot(trajectory_y)
subplot(2,1,2)
plot(trajectory_y_true)
% 
% figure  
% subplot(2,1,1)
% plot(theta)
% subplot(2,1,2)
% plot(theta_true)

figure  
subplot(3,1,1)
plot(covariance_x)
subplot(3,1,2)
plot(covariance_y)
subplot(3,1,3)
plot(covariance_theta)

figure  
subplot(2,1,1)
plot(bearing_array(:,4))
subplot(2,1,2)
plot(bearing_array_true(:,4))

% figure
% subplot(2,1,1)
% plot(command_array(:,1))
% subplot(2,1,2)
% plot(command_array(:,2))

% figure
% plot(model_noise_array)


%% Plot animated trajectory
figure('Name', 'Animated');
ylim([-10 100]);
xlim([-1 100]);

for k=2350:size(trajectory_x,2)
     cla
     hold on
%   set(hPlot, 'XData', trajectory_x(k), 'YData', trajectory_y(k))
    plot(trajectory_x(k), trajectory_y(k), 'o', 'MarkerSize', 8);
    plot([trajectory_x(k), trajectory_x(k) + 2*cos(theta(k))], [trajectory_y(k), trajectory_y(k) + 2*sin(theta(k))]);
    plot(trajectory_x_true(k), trajectory_y_true(k), 'x', 'MarkerSize', 8);
    plot([trajectory_x_true(k), trajectory_x_true(k) + 2*cos(theta_true(k))], [trajectory_y_true(k), trajectory_y_true(k) + 2*sin(theta_true(k))]);

    ylim([-10 100]);
    xlim([-1 100]);
    pause(0.1);
end

% array = [];
% noise = mvnrnd([0,0,0], Q)';
