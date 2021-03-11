function updated_covariance = predict_covariance(X, old_covariance, noise_covariance, u, dt)

v = u(1);
w = u(2);

F = [ 1, 0, -v*dt*sin(X(3)); 0, 1, v*dt*cos(X(3)); 0, 0, 1];

updated_covariance = F*old_covariance*F' + noise_covariance;

end

