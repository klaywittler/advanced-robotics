function [vel, omg] = estimate_vel(sensor, varargin)
%ESTIMATE_VEL 6DOF velocity estimator
%   sensor - struct stored in provided dataset, fields include:
%          - is_ready: logical, indicates whether sensor data is valid; if the
%                      sensor data is invalid, return empty arrays for vel, omg
%          - t: timestamp
%          - rpy, omg, acc: imu readings (you should not use these in this phase)
%          - img: uint8, 240x376 grayscale image
%          - id: 1xn ids of detected tags; if no tags are present return empty
%                arrays for vel, omg
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
%              estimate_vel_handle = ...
%                  @(sensor) estimate_vel(sensor, your personal input arguments);
%   vel - 3x1 velocity of the quadrotor in world frame
%   omg - 3x1 angular velocity of the quadrotor
persistent tracker t prevPoints

if isempty(sensor.id) || sensor.is_ready ~= 1
%     pos = [];
%     q = [];
    vel = zeros(3,0);
    omg = zeros(3,0);
elseif isempty(tracker)    
    points = detectHarrisFeatures(sensor.img);
    tracker = vision.PointTracker;
    initialize(tracker,points.Location,sensor.img)
    prevPoints = points.Location;
    t = sensor.t;
    vel = zeros(3,1);
    omg = zeros(3,1);
else
    K = varargin{1};
    pA = varargin{2}(:,:,sensor.id + 1);
    pA = reshape(permute(pA,[1,3,2]),[2,5*numel(sensor.id)]);
    
%     dt = lowpass(sensor.t - t,1,2.2);
%     dt = sensor.t - t;
    dt = 0.0205;
    [points,point_validity] = tracker(sensor.img);
    
    valid_points = (K\[points(point_validity,:), ones(numel(points(point_validity,1)),1)]')';
    prev_valid_points = (K\[prevPoints(point_validity,:), ones(numel(prevPoints(point_validity,1)),1)]')';
    dp = (valid_points(:,1:2) - prev_valid_points(:,1:2))/dt;
    
    [R,T] = getTransformation(sensor,pA,K);
%     R = varargin{3}*R;
%     T = varargin{3}*T + varargin{4};
    R = varargin{3}'*R;
    T = varargin{3}'*(T - varargin{4});
    [v, inliers] = motionRANSAC(valid_points(:,1:2),dp,K,R,T);
    
    if sum(point_validity) < 500
        disp('adding points')
        corners = detectHarrisFeatures(sensor.img);
        points = corners.Location;
        setPoints(tracker,points);   
    end
    vel = v(1:3);
    omg = v(4:6);
    
    t = sensor.t;
    prevPoints = points;
end

end
