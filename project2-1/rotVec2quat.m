function [q] = rotVec2quat(r)
%ROTVEC2QUAT Summary of this function goes here
%   Detailed explanation goes here
angle = norm(r);
axis = r/angle;
q = [cos(angle/2);axis*sin(angle/2)];
end

