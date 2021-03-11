function [updated_state, updated_covariance, K] = update_step(X, landmark, range, bearing,...
                                                                range_true, bearing_true, covariance, N)

y = [range_true; bearing_true] - [range; bearing];

% Jacobian of the measurement model
H = [(X(1) - landmark(1))/range, (X(2) - landmark(2))/range, 0; ...
    (landmark(2) - X(2))/range^2, (X(1) - landmark(1))/range^2, -1];

S = H*covariance*H'+ N;
K = covariance*H'*pinv(S);

delta = K*y;
updated_state = X + delta;
updated_covariance = (eye(3) - K*H)*covariance;

end

