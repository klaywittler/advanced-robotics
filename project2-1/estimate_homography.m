function [ H ] = estimate_homography(video_pts, logo_pts)
% est_homography estimates the homography to transform each of the
% video_pts into the logo_pts
% Inputs:
%     video_pts: a 2x4 matrix of corner points in the video
%     logo_pts: a 2x4 matrix of logo points that correspond to video_pts
% Outputs:
%     H: a 3x3 homography matrix such that logo_pts ~ H*video_pts
% Written for the University of Pennsylvania's Robotics:Perception course

% YOUR CODE HERE
A = zeros(numel(video_pts(:,1))*numel(video_pts(1,:)),9);
for i=1:numel(video_pts(1,:))
    ax = [-video_pts(1,i), -video_pts(2,i), -1, 0,0,0, video_pts(1,i)*logo_pts(1,i), video_pts(2,i)*logo_pts(1,i), logo_pts(1,i)];
    ay = [0,0,0, -video_pts(1,i), -video_pts(2,i), -1, video_pts(1,i)*logo_pts(2,i), video_pts(2,i)*logo_pts(2,i), logo_pts(2,i)];
    A(2*i-1:2*i,:) = [ax;ay];
end

[~, ~ , V] = svd (A) ;


H = reshape(V(:,end),[3,3])';

end

