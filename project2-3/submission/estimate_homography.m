function [ H ] = estimate_homography(video_pts, logo_pts)
% estimate_homography estimates the homography to transform each of the
% video_pts into the logo_pts
% Inputs:
%     video_pts: a 2x4 matrix of corner points in the video
%     logo_pts: a 2x4 matrix of logo points that correspond to video_pts
% Outputs:
%     H: a 3x3 homography matrix such that logo_pts ~ H*video_pts
% Written for the University of Pennsylvania's Robotics:Perception course

zr0 = zeros(numel(video_pts(1,:)),1);
o1 = ones(numel(video_pts(1,:)),1);
A = [-video_pts(1,:)', -video_pts(2,:)', -o1, zr0, zr0, zr0, video_pts(1,:)'.*logo_pts(1,:)', video_pts(2,:)'.*logo_pts(1,:)', logo_pts(1,:)';
    zr0 ,zr0 ,zr0 , -video_pts(1,:)', -video_pts(2,:)', -o1, video_pts(1,:)'.*logo_pts(2,:)', video_pts(2,:)'.*logo_pts(2,:)', logo_pts(2,:)'];

[~, ~ , V] = svd (A) ;

H = reshape(V(:,end)./V(end,end),[3,3])';

end

