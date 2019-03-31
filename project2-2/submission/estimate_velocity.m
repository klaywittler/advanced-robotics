function [v] = estimate_velocity(dp,p,A)
%ESTIMATE_VELOCITY Summary of this function goes here
%   Detailed explanation goes here
dp = reshape(dp', [numel(dp),1]);
v = A\dp;
end

