function [qcong] = quatCong(q)
%QUATCONG Summary of this function goes here
%   Detailed explanation goes here   
    qcong = [q(1); -q(2:end)];
end

