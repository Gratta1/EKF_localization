%% 2D EKF for localization of mobile robot and known marker association
%% Robot motion model
% state vector X = [x, y, theta, lx, ly]
% input u = [v, w] v: linear velocity; w: angular velocity
% dt = delta of time between inputs
% Model of a unicycle with wheel radius = 1
clc
clear

% Create random input for the robot

vmin= 1;
vmax= 3;
v=vmin+rand()*(vmax-vmin);
wmin= -5;
wmax= 5;
w=wmin+rand()*(wmax-wmin);

% Create noise and state of the robot
Q = [ 0.0005, 0, 0, 0, 0; 0, 0.0005, 0, 0, 0; 0, 0, 0.00005, 0, 0; 0, 0, 0, 0, 0;0, 0, 0, 0, 0 ];
N = [ 0.002, 0; 0, 0.00002 ];
noise = mvnrnd([0,0,0, 0, 0], Q)';
landmark = [30, 18];
% n_landmarks = size(landmark, 1);
X = [0; 0; 0; 31; 19];
X_true = X + noise - [0; 0; 0; 1; 1];
dt = 0.01;
u = [v;w];
trajectory_x = X(1);
trajectory_y = X(2);
trajectory_x_true = X_true(1);
trajectory_y_true = X_true(2);
update = [];
theta = [];
theta_true = [];
update = [];
covariance_x = [];
covariance_y = [];
covariance_theta= [];

i = 0;
T = 0;
% rng('default')

covariance = Q;
covariance(4,4) = 0.0005;
covariance(5,5) = 0.0005;


%% Loop

while( T < 10)

% Create random input
v=vmin+rand()*(vmax-vmin);
w=wmin+rand()*(wmax-wmin);

u = [v,w];

% Create state and measure noise
noise = mvnrnd([0,0,0,0,0], Q)';
noise_measure = mvnrnd([0, 0], N)';

% Move forward the robot (adding noise) and prediction step
covariance = predict_covariance(X, covariance, Q, u, dt);
X_true = move_forward(X_true, u, dt, noise);
X = prediction_step(X, u, dt);

% Measure the distance from the landmark and make the measurement step for
% EKF
[range_true, bearing_true] = measure_step(X_true, landmark, noise_measure);
[range, bearing] = measure_prediction(X, landmark);
update(1) = 0;
update(2) = 0;

if mod(i, 1) == 0 && i~=0
[X, covariance, K] = update_step(X, landmark, range, bearing, range_true, bearing_true, covariance, N);
update(i,1) = X(1);
update(i,2) = X(2);
end

% Update trajectories for plot
trajectory_x = [trajectory_x, X(1)];
trajectory_y = [trajectory_y, X(2)];
trajectory_x_true = [trajectory_x_true, X_true(1)];
trajectory_y_true = [trajectory_y_true, X_true(2)];
theta = [theta, X(3)];
theta_true = [theta_true; X_true(3)];
update = [];
covariance_x = [covariance_x; covariance(1,1)];
covariance_y = [covariance_y; covariance(2,2)];
covariance_theta= [covariance_theta; covariance(3,3)];

% bearing_array_true = [bearing_array_true, bearing_true];
% bearing_array = [bearing_array, bearing];

i = i+1;
T = T+dt;


end

%% Plot trajectories and landmark position
figure  
hold on 
plot(trajectory_x_true, trajectory_y_true)
plot(trajectory_x, trajectory_y)
plot(landmark(1), landmark(2), 'x')
plot(X(4), X(5), 'x')
axis equal

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

figure  
subplot(2,1,1)
plot(theta)
subplot(2,1,2)
plot(theta_true)

figure  
subplot(3,1,1)
plot(covariance_x)
subplot(3,1,2)
plot(covariance_y)
subplot(3,1,3)
plot(covariance_theta)