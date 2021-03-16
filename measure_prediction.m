function [range, bearing] = measure_prediction(X, landmark)

n_landmark = size(landmark, 1);
range = [];
bearing = [];


for i=1:n_landmark
    
range(i) = sqrt((landmark(i,1) - X(1))^2 + (landmark(i,2) - X(2))^2);
bearing(i) = atan2( landmark(i,2) - X(2), landmark(i,1) - X(1) ) - X(3);

end


end
