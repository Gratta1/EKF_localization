function [range, bearing] = measure_step(X, landmark, noise)

range = sqrt((landmark(1) - X(1))^2 + (landmark(2) - X(2))^2) + noise(1);
bearing = wrapToPi(atan2(landmark(2) - X(2), landmark(1) - X(1))) -  wrapToPi(X(3)) + noise(2);

end

