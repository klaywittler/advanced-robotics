function [R,T] = getTransformation(sensor,pA,K)
%GETTRANSFORMATION Summary of this function goes here
%   Detailed explanation goes here
    p = K\[sensor.p0,sensor.p1,sensor.p2,sensor.p3,sensor.p4;ones(1,5*numel(sensor.id))];
    p = p(1:2,:);
    H = estimate_homography(pA,p);
    [R,T] = estimate_transformation(H);
end

function [ H ] = estimate_homography(video_pts, logo_pts)
% estimate_homography estimates the homography to transform each of the
% video_pts into the logo_pts
% Inputs:
%     video_pts: a 2x4 matrix of corner points in the video
%     logo_pts: a 2x4 matrix of logo points that correspond to video_pts
% Outputs:
%     H: a 3x3 homography matrix such that logo_pts ~ H*video_pts
% Written for the University of Pennsylvania's Robotics:Perception course

A = zeros(numel(video_pts(:,1))*numel(video_pts(1,:)),9);
for i=1:numel(video_pts(1,:))
    ax = [-video_pts(1,i), -video_pts(2,i), -1, 0,0,0, video_pts(1,i)*logo_pts(1,i), video_pts(2,i)*logo_pts(1,i), logo_pts(1,i)];
    ay = [0,0,0, -video_pts(1,i), -video_pts(2,i), -1, video_pts(1,i)*logo_pts(2,i), video_pts(2,i)*logo_pts(2,i), logo_pts(2,i)];
    A(2*i-1:2*i,:) = [ax;ay];
end

[~, ~ , V] = svd (A) ;


H = reshape(V(:,end)./V(end,end),[3,3])';

end

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
