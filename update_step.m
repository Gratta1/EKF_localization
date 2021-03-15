function [updated_state, updated_covariance, K] = update_step(X, landmark, range, bearing,...
                                                                range_true, bearing_true, covariance, N)
sum_1 = 0;
sum_2 = 0;
n_landmark = size(landmark, 1);

for i = 1:n_landmark
    
y = [range_true(i); bearing_true(i)] - [range(i); bearing(i)];

% Jacobian of the measurement model
H = [(X(1) - landmark(i,1))/range(i), (X(2) - landmark(i,2))/range(i), 0; ...
    (landmark(i,2) - X(2))/range(i)^2, (X(1) - landmark(i,1))/range(i)^2, -1];

S = H*covariance*H'+ N;
K = covariance*H'*pinv(S);

sum_1 =  sum_1 + K*y;
sum_2 = sum_2 + K*H;

end

if sum_1/n_landmark > 0.2
    ciao = 2 %for debug
end

updated_state = X + sum_1/n_landmark;
updated_covariance = (eye(3) - sum_2/n_landmark)*covariance;

end

