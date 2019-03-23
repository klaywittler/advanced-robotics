function [q] = rot2quat(R)
%ROT2QUAT Summary of this function goes here
%   Detailed explanation goes here

angle = acos((trace(R) - 1)/2);
% [V,D] = eig(R);
axis_hat = 1/(2*sin(angle))*(R-R');
axis = veemap(axis_hat);

q =[cos(angle/2);sin(angle/2)*axis];

end

function w = veemap(R)
    n = size(R');
    if n(1) == 3
    w = [-R(2,3); R(1,3); -R(1,2)];
    else 
    w = [0 -R(3) R(2); R(3) 0 -R(1); -R(2) R(1) 0];
    end
end
