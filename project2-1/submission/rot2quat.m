function [q] = rot2quat(R)
%ROT2QUAT converts a rotation matrix to quaternion representation
% not robust to singularities as it converts to axis-angle first
%   Input:
%       R: 3x3 rotation matrix
%   Output:
%       q: 4x1 unit quaternion

angle = acos((trace(R) - 1)/2);
axis_hat = (R-R')/(2*sin(angle));
axis = veemap(axis_hat);

q =[cos(angle/2);sin(angle/2)*axis];

end

function w = veemap(R)
    n = size(R');
    if n(1) == 3 && n(2) == 3
        w = [-R(2,3); R(1,3); -R(1,2)];
    else 
        w = [0 -R(3) R(2); R(3) 0 -R(1); -R(2) R(1) 0];
    end
end
