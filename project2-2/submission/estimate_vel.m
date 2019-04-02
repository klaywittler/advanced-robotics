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
persistent tracker prevPoints t stm1 vm1 omgm1

if isempty(sensor.id) || sensor.is_ready ~= 1
    vel = zeros(3,0);
    omg = zeros(3,0);
elseif isempty(tracker)    
    points = detectFASTFeatures(sensor.img);
    points = points.selectStrongest(50);
    tracker = vision.PointTracker;
    initialize(tracker,points.Location,sensor.img)
    prevPoints = points.Location;
    t = sensor.t;
    stm1 = sensor.t;
    vm1 = zeros(3,1);
    omgm1 = zeros(3,1);
    vel = zeros(3,1); % make zeros(3,0); for submission 
    omg = zeros(3,1);
else
%     dt = 0.0205; % 50 Hz
    alpha = 0.42;
    beta = 0.70;
    dt = sensor.t - t;
    st = alpha*dt + (1-alpha)*stm1;
    
    K = varargin{1};
    pA = varargin{2}(:,:,sensor.id + 1);
    pA = reshape(permute(pA,[1,3,2]),[2,5*numel(sensor.id)]);
    
    [points,point_validity] = tracker(sensor.img);
    
    o1 = ones(numel(points(point_validity,1)),1);
    valid_points = (K\[points(point_validity,:), o1]')';
    prev_valid_points = (K\[prevPoints(point_validity,:), o1]')';
%     valid_points = (points(point_validity,:)- 1/K(1,3))/K(1,1);
%     prev_valid_points = (prevPoints(point_validity,:)- 1/K(2,3))/K(2,2);
    dp = (valid_points(:,1:2) - prev_valid_points(:,1:2))/st;
    
    p = K\[sensor.p0,sensor.p1,sensor.p2,sensor.p3,sensor.p4;ones(1,5*numel(sensor.id))];
    p = p(1:2,:);
    H = estimate_homography(pA,p);
    [R,~] = estimate_transformation(H);
    [v, ~] = motionRANSAC(prev_valid_points(:,1:2),dp,H);
       
    if sum(point_validity) < 20
        corners = detectFASTFeatures(sensor.img);
        corners =  corners.selectStrongest(50);
        points = corners.Location;
        setPoints(tracker,points);   
    end
    
    
    vel = beta*R'*v(1:3) + (1-beta)*vm1;
    omg = beta*R'*v(4:6) + (1-beta)*omgm1;
    
    vm1 = vel;
    omgm1 = omg;
    prevPoints = points;
    t = sensor.t;
    stm1 = st;
end

end
