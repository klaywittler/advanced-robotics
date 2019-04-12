function [q] = eulzxy2quat(ang)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    R = eulzxy2rot(ang);
    q = rot2quat(R);
end

