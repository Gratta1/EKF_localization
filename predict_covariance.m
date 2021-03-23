function updated_covariance = predict_covariance(X, old_covariance, noise_covariance, u, dt, landmark)

n_landmark = size(landmark, 1);
v = u(1);
w = u(2);

F = zeros(3, 3 + 2*n_landmark);

g = [ 0, 0, -v*dt*sin(X(3));
      0, 0, v*dt*cos(X(3));
      0, 0, 0;];

G = eye(3+2*n_landmark)+F'*g*F;
updated_covariance = G*old_covariance*G' + noise_covariance;

end

