function [updated_state, updated_covariance, K] = update_step(X, landmark, range, bearing,...
                                                                range_true, bearing_true, covariance, N)
sum_1 = 0;
sum_2 = 0;
n_landmark = size(landmark, 1);
active_landmark = 0;

for i = 1:n_landmark
    
   if abs(bearing(i) - bearing_true(i)) < 1
        y = [range_true(i); bearing_true(i)] - [range(i); bearing(i)];

        % Jacobian of the measurement model
        H = [(X(1) - landmark(i,1))/range(i), (X(2) - landmark(i,2))/range(i), 0; ...
            (landmark(i,2) - X(2))/range(i)^2, (X(1) - landmark(i,1))/range(i)^2, -1];

        S = H*covariance*H'+ N;
        K = covariance*H'*pinv(S);

        sum_1 =  sum_1 + K*y;
        sum_2 = sum_2 + K*H;
        active_landmark = active_landmark + 1;
   end
end


updated_state = X + sum_1/active_landmark;
updated_covariance = (eye(3) - sum_2/active_landmark)*covariance;


end

