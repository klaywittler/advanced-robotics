function [v] = estimate_velocity(dp,p,A)
%ESTIMATE_VELOCITY Summary of this function goes here
%   Detailed explanation goes here

% A = zeros(numel(dp),6);
% for i=1:2:numel(dp)
% A(i:i+1,:) = [-1*z, 0, p(1)*z, p(1)*p(2), -(1 + p(1)^2), p(2);
%                 0, -1*z, p(2)*z, 1+p(2)^2, -p(1)*p(2), -p(1)];
% end
dp = reshape(dp', [numel(dp),1]);
v = A\dp;
end

