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
   sPrev = 100*eye(15);
end
if isempty(xPrev)
    xPrev = zeros(15,1);
end

if sensor.is_ready ~= 1
    x = xPrev;
else
    dt = 0.0205; 
    u = [sensor.acc; sensor.omg];
    [x,S] = prediction(xPrev,sPrev,u,dt);

    if ~isempty(sensor.id)
        [pos, ang, vel, ~] = estimate_state(sensor, varargin{:});
        z = [pos;ang;vel];
        [x, S] = measurement(x,S,z);
    end
    
    xPrev = x;
    sPrev = S;
    
end
q = eulzxy2quat(x(4:6));
X = [x(1:3);x(7:9);q];
Z = x;

end

function [xNext, SNext] = prediction(x,S,u,dt)
    Q = 10*eye(12);
    n = zeros(12,1);
    [F,V,xdot] = getParameters2(x,u,n,dt);
    xNext = x + xdot*dt;

    SNext = F*S*F' + V*Q*V';
end

function [xNext, SNext] = measurement(x,S,z)
    R = 0.1*eye(9);

    C = [eye(3) zeros(3) zeros(3) zeros(3) zeros(3);
        zeros(3) eye(3) zeros(3) zeros(3) zeros(3);
        zeros(3) zeros(3) eye(3) zeros(3) zeros(3)];

    K = S*C'/(C*S*C' + R);
    xNext = x + K*(z-C*x);
    SNext = S - K*C*S;
end

