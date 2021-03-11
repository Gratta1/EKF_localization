%% Prediction model of the unicycle
function X = prediction_step(X_prev, u, dt)

v = u(1);
w = u(2);

X = X_prev + [v*dt*cos(X_prev(3)); v*dt*sin(X_prev(3)); w*dt];

end