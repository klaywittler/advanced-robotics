function [R,T] = estimate_transformation(r)
%estimate_transformation utilizes planar case in z-axis and homography to
%find rotation matrix and translation vector
%   Input:
%       r: 3x3 estimated homography between two images
%   Output:
%       R: 3x3 rotation matrix such that p2 = Rp1
%       T: 3x1 translation vector p2 = p1 + T
Rhat = [r(:,1) r(:,2) cross(r(:,1),r(:,2))];
[U,~,V] = svd(Rhat);
R = U*diag([1,1,det(U*V')])*V';
T = r(:,3)/(0.5*norm(r(:,1)) + 0.5*norm(r(:,2)));
end

