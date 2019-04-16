function [pos,ang,vel,omg] = estimate_state(sensor, varargin)
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
persistent tracker prevPoints vOld omgOld
nPoints = 113;

if isempty(sensor.id) || sensor.is_ready ~= 1
    pos = zeros(3,0);
    ang = zeros(3,0);
    vel = zeros(3,0);
    omg = zeros(3,0);
elseif isempty(tracker)    
    points = detectFASTFeatures(sensor.img);
    points = points.selectStrongest(nPoints);
%     points = selectUniform(points,nPoints,size(sensor.img));
    tracker = vision.PointTracker('MaxBidirectionalError',2);
    initialize(tracker,points.Location,sensor.img)
    prevPoints = points.Location;
    vOld = zeros(3,1);
    omgOld = zeros(3,1);
    vel = zeros(3,1);
    omg = zeros(3,1);
    
    [pos, ang, ~, ~, ~] = estimate_pose(sensor, varargin{:});
    
else
    alpha = 0.40;
    beta = 0.95;  
%     alpha = 1;
%     beta = 1;
    
    st = 0.0205;
    Kinv = varargin{1};
    [pos, ang, H, R, T] = estimate_pose(sensor, varargin{:});
    
    
    [points,validity] = tracker(sensor.img);
    
    o1 = ones(numel(points(validity,1)),1);
    valid_points = (Kinv*[points(validity,:), o1]')';
    prev_valid_points = (Kinv*[prevPoints(validity,:), o1]')';
    dp = (valid_points(:,1:2) - prev_valid_points(:,1:2))/st;    
    
    v = motionRANSAC(prev_valid_points(:,1:2),dp, H, R , T);
       
    if sum(validity) < 0.8*nPoints
        corners = detectFASTFeatures(sensor.img);
        corners =  corners.selectStrongest(nPoints);
        points = corners.Location;
        setPoints(tracker,points);   
    end
    vel = R'*v(1:3);
    omg = R'*v(4:6);
    
    vel = alpha*vel  + (1-alpha)*vOld;
    omg = beta*omg + (1-beta)*omgOld;
    
    vOld = vel;
    omgOld = omg;
    prevPoints = points;
end

end
