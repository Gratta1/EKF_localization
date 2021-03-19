function [range, bearing] = measure_prediction(X, landmark)

n_landmark = size(landmark, 1);
range = [];
bearing = [];


for i=1:n_landmark
    
range(i) = sqrt((X(4) - X(1))^2 + (X(5) - X(2))^2);
bearing(i) = wrapToPi(atan2(X(5) - X(2), X(4) - X(1)) - X(3));

end

end


