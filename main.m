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
Q = [ 0.0009, 0, 0, 0, 0; 0, 0.0009, 0, 0, 0; 0, 0, 0.0005, 0, 0; 0, 0, 0, 0, 0;0, 0, 0, 0, 0 ];
N = [ 0.02, 0; 0, 0.0002 ];
noise = mvnrnd([0,0,0, 0, 0], Q)';
landmark = [30, 18];
% n_landmarks = size(landmark, 1);
X = [0; 0; 0; 32; 20];
X_true = X + noise - [0; 0; 0; 2; 2];
dt = 0.01;
u = [v;w];
trajectory_x = X(1);
trajectory_y = X(2);
trajectory_x_true = X_true(1);
trajectory_y_true = X_true(2);
update = [];
i = 0;

T = 0;
% rng('default')

covariance = Q;
covariance(4,4) = 1;
covariance(5,5) = 1;


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

if mod(i, 100) == 0 && i~=0
[X, covariance, K] = update_step(X, landmark, range, bearing, range_true, bearing_true, covariance, N);
update(i,1) = X(1);
update(i,2) = X(2);
end

% Update trajectories for plot
trajectory_x = [trajectory_x, X(1)];
trajectory_y = [trajectory_y, X(2)];
trajectory_x_true = [trajectory_x_true, X_true(1)];
trajectory_y_true = [trajectory_y_true, X_true(2)];
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
% plot(update(:,1), update(:,2), 'x')
axis equal
