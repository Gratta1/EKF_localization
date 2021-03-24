%% Prediction model of the unicycle
function X = move_forward(X_prev, u, dt, noise, n_landmark)

v = u(1);
w = u(2);

X = X_prev + [v*dt*cos(X_prev(3)); v*dt*sin(X_prev(3)); w*dt; zeros(2*n_landmark,1)] + noise;

end