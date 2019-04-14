function [X, Z] = ekf1(sensor, vic, varargin)
% EKF1 Extended Kalman Filter with Vicon velocity as inputs
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
%   vic    - struct for storing vicon linear velocity in world frame and
%            angular velocity in body frame, fields include
%          - t: vicon timestamp
%          - vel = [vx; vy; vz; wx; wy; wz]
%   varargin - any variables you wish to pass into the function, could be
%              a data structure to represent the map or camera parameters,
%              your decision. But for the purpose of testing, since we don't
%              know what inputs you will use, you have to specify them in
%              init_script by doing
%              ekf1_handle = ...
%                  @(sensor, vic) ekf1(sensor, vic, your input arguments);
%
% OUTPUTS:
% X - nx1 state of the quadrotor, n should be greater or equal to 6
%     the state should be in the following order
%     [x; y; z; qw; qx; qy; qz; other states you use]
%     we will only take the first 7 rows of X
% OPTIONAL OUTPUTS:
% Z - mx1 measurement of your pose estimator, m shoulb be greater or equal to 7
%     the measurement should be in the following order
%     [x; y; z; qw; qx; qy; qz; other measurement you use]
%     note that this output is optional, it's here in case you want to log your
%     measurement
persistent xPrev sPrev

if isempty(sPrev)
   sPrev = 100*eye(9);
end
if isempty(xPrev)
    xPrev = zeros(9,1);
end

dt = 0.01; 
[x,S] = prediction(xPrev,sPrev,vic.vel,dt);

if ~isempty(sensor.id) && sensor.is_ready == 1
    [pos, ang, ~, ~, ~] = estimate_pose(sensor, varargin{:});
    z = [pos;ang];
    [x, S] = measurement(x,S,z); 
end
      
xPrev = x;
sPrev = S;
    
q = eulzxy2quat(x(4:6));
X = [x(1:3);q];
Z = x;

end

function [xtp1, Stp1] = prediction(x,S,u,dt)
Q = 0.01*eye(9);
n = zeros(9,1);
[F,V,xdot] = getParameters1(x,u,n,dt);
xtp1 = x + xdot*dt;

Stp1 = F*S*F' + V*Q*V';

end

function [xtp1, Stp1] = measurement(x,S,z)
R = 0.01*eye(6);

C = [eye(3) zeros(3) zeros(3);
    zeros(3) eye(3) zeros(3)];

K = S*C'/(C*S*C' + R);
xtp1 = x + K*(z-C*x);
Stp1 = S - K*C*S;

end
