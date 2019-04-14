function [pos, ang, H, R, T] = estimate_pose(sensor, varargin)
%ESTIMATE_POSE 6DOF pose estimator based on apriltags
%   sensor - struct stored in provided dataset, fields include:
%          - is_ready: logical, indicates whether sensor data is valid
%          - rpy, omg, acc: imu readings (you should not use these in this phase)
%          - img: uint8, 240x376 grayscale image
%          - id: 1xn ids of detected tags, if no tags are present return empty
%                arrays for pos, q
%          - p0, p1, p2, p3, p4: 2xn pixel position of center and
%                                four corners of detected tags
%            Y
%            ^ P3 == P2
%            | || P0 ||
%            | P4 == P1
%            o---------> X
%   varargin - any variables you wish to pass into the function, could be
%              a data structure to represent the map or camera parameters,
%              your decision. But for the purpose of testing, since we don't
%              know what inputs you will use, you have to specify them in
%              init_script by doing
%              estimate_pose_handle = ...
%                  @(sensor) estimate_pose(sensor, your personal input arguments);
%               varargin{1} - 3x3 camera intrinsics matrix
%               varargin{2} - April tag map
%               varargin{3} - 3x3 rotation matrix from imu to camera
%               varargin{4} - 3x1 translation vector from camerat to imu
%   pos - 3x1 position of the quadrotor in world frame
%   q   - 4x1 quaternion of the quadrotor [w, x, y, z] where q = w + x*i + y*j + z*k

if isempty(sensor.id) || sensor.is_ready ~= 1
    pos = zeros(3,0);
    ang = zeros(3,0);
else
    Kinv = varargin{1};
    pA = varargin{2}(:,:,sensor.id + 1);
    pA = reshape(permute(pA,[1,3,2]),[2,5*numel(sensor.id)]);
    p = Kinv*[sensor.p0,sensor.p1,sensor.p2,sensor.p3,sensor.p4;ones(1,5*numel(sensor.id))];
    p = p(1:2,:);
    H = estimate_homography(pA,p);
    [R,T] = estimate_transformation(H);
    
    pC = varargin{3}*[0;0;0] + varargin{4};
    pW = R'*(pC - T);
    pos = pW(1:3);
    ang = rot2eulzxy(R'*varargin{3});
end

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

zr0 = zeros(numel(video_pts(1,:)),1);
o1 = ones(numel(video_pts(1,:)),1);
A = [-video_pts(1,:)', -video_pts(2,:)', -o1, zr0, zr0, zr0, video_pts(1,:)'.*logo_pts(1,:)', video_pts(2,:)'.*logo_pts(1,:)', logo_pts(1,:)';
    zr0 ,zr0 ,zr0 , -video_pts(1,:)', -video_pts(2,:)', -o1, video_pts(1,:)'.*logo_pts(2,:)', video_pts(2,:)'.*logo_pts(2,:)', logo_pts(2,:)'];

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
r3 = [r(2,1)*r(3,2)-r(3,1)*r(2,2); r(3,1)*r(1,2)-r(1,1)*r(3,2); r(1,1)*r(2,2)-r(2,1)*r(1,2)];
Rhat = [r(:,1) r(:,2) r3];
[U,~,V] = svd(Rhat);
R = U*diag([1,1,det(U*V')])*V';
T = r(:,3)/(0.5*sqrt(sum(r(:,1).^2)) + 0.5*sqrt(sum(r(:,2).^2)));
end
