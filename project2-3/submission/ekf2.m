function [X, Z] = ekf2(sensor, varargin)
% EKF2 Extended Kalman Filter with IMU as inputs
%
% INPUTS:
%   sensor - struct stored in provided dataset, fields include
%          - is_ready: logical, indicates whether sensor data is valid
%          - t: sensor timestamp
%          - rpy, omg, acc: imu readings
%          - img: uint8, 240x376 grayscale image
%          - id: 1xn ids of detected tags
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
%              ekf1_handle = ...
%                  @(sensor) ekf2(sensor, your input arguments);
%
% OUTPUTS:
% X - nx1 state of the quadrotor, n should be greater or equal to 9
%     the state should be in the following order
%     [x; y; z; vx; vy; vz; qw; qx; qy; qz; other states you use]
%     we will only take the first 10 rows of X
% OPTIONAL OUTPUTS:
% Z - mx1 measurement of your pose estimator, m shoulb be greater or equal to 6
%     the measurement should be in the following order
%     [x; y; z; qw; qx; qy; qz; other measurement you use]
%     note that this output is optional, it's here in case you want to log your
%     measurement

persistent xPrev sPrev

if isempty(sPrev)
   sPrev = 0.1*eye(15);
end
if isempty(xPrev)
    xPrev = zeros(15,1);
end

if isempty(sensor.id) || sensor.is_ready ~= 1
    X = zeros(10,1);
    Z = zeros(15,1);
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
    q = rot2eulzxy(R'*varargin{3});
    
    
    z = [pos;q];
    [x, S] = measurement(xPrev,sPrev,z);
    
    dt = 0.0205; 
    u = [sensor.acc; sensor.omg];
    [x,S] = prediction(x,S,u,dt);
    xPrev = x;
    sPrev = S;
    
    q = eulzxy2quat(x(4:6));
    X = [x(1:3);x(7:12);q];
    Z = x;
end

end

function [xtp1, Stp1] = prediction(x,S,u,dt)
Q = 0.01*eye(15);
n = zeros(15,1);
[F,V,xdot] = getParameters2(x,u,n,dt);
xtp1 = x + xdot*dt;

Stp1 = F*S*F' + V*Q*V';

end

function [xtp1, Stp1] = measurement(x,S,z)
R = 0.01*eye(12);

C = [eye(3) zeros(3) zeros(3) zeros(3);
    zeros(3) eye(3) zeros(3) zeros(3);
    zeros(3) zeros(3) eye(3) zeros(3)];

K = S*C'/(C*S*C' + R);
xtp1 = x + K*(z-C*x);
Stp1 = S - K*C*S;

end

