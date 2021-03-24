function [updated_state, updated_covariance, K] = update_step(X, landmark, range, bearing,...
                                                                range_true, bearing_true, covariance, N)
sum_1 = 0;
sum_2 = 0;
n_landmark = size(landmark, 1);

for i = 1:n_landmark
    if abs(bearing(i) - bearing_true(i)) < 1
y = [range_true(i); bearing_true(i)] - [range(i); bearing(i)];

% Jacobian of the measurement model
H = [(X(1) - X(2+2*i))/range(i), (X(2) - X(2+2*i+1))/range(i), 0, (X(2+2*i) - X(1))/range(i), (X(2+2*i+1) - X(2))/range(i); ...
    (X(2+2*i+1) - X(2))/range(i)^2, (X(1) - X(2+2*i))/range(i)^2, -1, (X(2) - X(2+2*i+1))/range(i)^2, (X(2+2*i) - X(1))/range(i)^2 ];

F_j = zeros(5, 3+2*n_landmark);
F_j(1,1) = 1;
F_j(2,2) = 1;
F_j(3,3) = 1;
F_j(4,2+2*i) = 1;
F_j(5,2+2*i+1) = 1;
G = H*F_j;
S = G*covariance*G'+ N;
K = covariance*G'*pinv(S);

X = X + K*y;
covariance = (eye(3+2*n_landmark)- K*G)*covariance;
    end
end


updated_state = X;
updated_covariance = covariance;


end

