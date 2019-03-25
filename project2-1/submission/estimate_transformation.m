function [R,T] = estimate_transformation(r)
%ESTIMATE_TRANSFORMATION Summary of this function goes here
%   Detailed explanation goes here
Rhat = [r(:,1) r(:,2) cross(r(:,1),r(:,2))];
[U,~,V] = svd(Rhat);
R = U*diag([1,1,det(U*V')])*V';
T = r(:,3)/(0.5*norm(r(:,1)) + 0.5*norm(r(:,2)));
end

