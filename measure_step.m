function [range, bearing] = measure_step(X, landmark, noise)

n_landmark = size(landmark, 1);
range = [];
bearing = [];

for i=1:n_landmark
    
    range(i) = sqrt((landmark(i, 1) - X(1))^2 + (landmark(i, 2) - X(2))^2) + noise(1);
    bearing(i) = WrapTo2Pi(wrapToPi(atan2(landmark(i, 2) - X(2), landmark(i, 1) - X(1)) -  X(3) + noise(2)));

end

end

