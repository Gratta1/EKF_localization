function [range, bearing] = measure_prediction(X, landmark)

range = sqrt((landmark(1) - X(1))^2 + (landmark(2) - X(2))^2);
bearing = wrapToPi(atan2(landmark(2) - X(2), landmark(1) - X(1))) - wrapToPi(X(3));

end


