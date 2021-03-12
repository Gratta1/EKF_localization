%% 2D EKF for localization of mobile robot and known marker association
%% Robot motion model
% state vector X = [x, y, theta]
% input u = [v, w] v: linear velocity; w: angular velocity
% dt = delta of time between inputs
% Model of a unicycle with wheel radius = 1
clc
clear

% Create random input for the robot
vmin=1;
vmax=3;
v=vmin+rand()*(vmax-vmin);
wmin= - 0.3;
wmax= 0.3;
w=wmin+rand()*(wmax-wmin);

% Create noise and state of the robot
Q = [ 0.0005, 0, 0; 0, 0.0005, 0; 0, 0, 0.00005 ];
N = [ 0.0002, 0; 0, 0.00002 ];
noise = mvnrnd([0,0,0], Q)';
landmark = [1000, 1000];
X = [0; 0; 0];
X_true = X + noise;
dt = 0.001;
u = [v;w];
trajectory_x = X(1);
trajectory_y = X(2);
trajectory_x_true = X_true(1);
trajectory_y_true = X_true(2);

i = 0;
T = 0;
% rng('default')

covariance = Q;
bearing_array = [];
bearing_array_true = [];

%% Loop
while( T < 10)

% Create random input
v=vmin+rand()*(vmax-vmin);
w=wmin+rand()*(wmax-wmin);
u = [v,w];

% Create state and measure noise
noise = mvnrnd([0,0,0], Q)';
noise_measure = mvnrnd([0, 0], N)';

% Move forward the robot (adding noise) and prediction step
covariance = predict_covariance(X, covariance, Q, u, dt);
X_true = move_forward(X_true, u, dt, noise);
X = prediction_step(X, u, dt);

% Measure the distance from the landmark and make the measurement step for
% EKF
[range_true, bearing_true] = measure_step(X_true, landmark, noise_measure);
[range, bearing] = measure_prediction(X, landmark);

[X, covariance] = update_step(X, landmark, range, bearing, range_true, bearing_true, covariance, N);

% Update trajectories for plot
trajectory_x = [trajectory_x, X(1)];
trajectory_y = [trajectory_y, X(2)];
trajectory_x_true = [trajectory_x_true, X_true(1)];
trajectory_y_true = [trajectory_y_true, X_true(2)];
bearing_array_true = [bearing_array_true, bearing_true];
bearing_array = [bearing_array, bearing];

i = i+1;
T = T+dt;
%  if abs(X(1)-X_true(1)) > 3 || abs(X(2)-X_true(2)) > 3
%     break
%  end
%  
end

%% Plot trajectories and landmark position
figure  
hold on 
plot(trajectory_x_true, trajectory_y_true)
% plot(landmark(1), landmark(2), 'x')
plot(trajectory_x, trajectory_y)
axis equal
