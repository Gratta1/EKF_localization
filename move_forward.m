%% Prediction model of the unicycle
function X = move_forward(X_prev, u, dt, noise)

v = u(1);
w = u(2);

X = X_prev + [v*dt*cos(X_prev(3)); v*dt*sin(X_prev(3)); w*dt; 0; 0] + noise;

end