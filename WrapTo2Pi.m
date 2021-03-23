function angle = WrapTo2Pi(old_angle)
%WRAPTO2PI Summary of this function goes here
%   Detailed explanation goes here
if old_angle < 0
    angle = 2*pi + old_angle;
else
    angle = old_angle;
end
end

