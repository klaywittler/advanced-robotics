function [F, M, trpy, drpy] = controller(qd, t, qn, params)
% CONTROLLER quadrotor controller
% The current states are:
%% Inputs:
%
% qd{qn}: state and desired state information for quadrotor #qn (qn
%         will be = 1 since we are only flying a single robot)
%
%  qd{qn}.pos, qd{qn}.vel   position and velocity
%  qd{qn}.euler = [roll;pitch;yaw]
%  qd{qn}.omega     angular velocity in body frame
% 
%  qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des  desired position, velocity, accel
%  qd{qn}.yaw_des, qd{qn}.yawdot_des
%
% t: current time
%    
% qn: quadrotor number, should always be 1
%    
% params: various parameters
%  params.I     moment of inertia
%  params.grav  gravitational constant g (9.8...m/s^2)
%  params.mass  mass of robot
%
%% Outputs:
%
% F: total thrust commanded (sum of forces from all rotors)
% M: total torque commanded
% trpy: thrust, roll, pitch, yaw (attitude you want to command!)
% drpy: time derivative of trpy
%
% Using these current and desired states, you have to compute the desired
% controls u, and from there F and M
%

% =================== Your code goes here ===================
K_p = [9,0,0;...
       0,9,0;...
       0,0,9];
K_d = 2*sqrt(K_p);
K_r = [10000,0,0;...
       0,10000,0;...
       0,0,10000];
K_w = 2*sqrt(K_r);

accel_des = qd{qn}.acc_des - K_d*(qd{qn}.vel - qd{qn}.vel_des) - K_p*(qd{qn}.pos - qd{qn}.pos_des);
F_des = params.mass * accel_des + [0;0;params.mass*params.grav];
R = eulzxy2rotmat(qd{qn}.euler);
u_1 = (R*[0;0;1]).'*F_des;
u_2 = [0;0;0];

if any(F_des)
    b3_des = F_des/norm(F_des);
    b2_des = cross(b3_des, [cos(qd{qn}.yaw_des); sin(qd{qn}.yaw_des); 0]);
    b2_des = b2_des/norm(b2_des);
    R_des = [cross(b2_des, b3_des), b2_des, b3_des];

    e_r = 0.5*veemap(R_des.'*R-R.'*R_des);
    u_2 = params.I*(-K_r*e_r.' - K_w*(qd{qn}.omega));
    ptpsi = rotmat2eulzxy(R_des);
end
% ==============================

% Desired roll, pitch and yaw (in rad). In the simulator, those will be *ignored*.
% When you are flying in the lab, they *will* be used (because the platform
% has a built-in attitude controller). Best to fill them in already
% during simulation.

phi_des   = ptpsi(1);
theta_des = ptpsi(2);
psi_des   = ptpsi(3);

%
%
%  
% Thrust
F    = u_1;       % This should be F = u(1) from the project handout

% Moment
M    = u_2;     % note: params.I has the moment of inertia

% =================== Your code ends here ===================

% Output trpy and drpy as in hardware
trpy = [F, phi_des, theta_des, psi_des];
drpy = [0, 0,       0,         0];

end

%
% ------------------------------------------------------------
%    should you decide to write a geometric controller,
%    the following functions should come in handy
%

function m = eulzxy2rotmat(ang)
    phi   = ang(1);
    theta = ang(2);
    psi   = ang(3);
    
    m = [[cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta), -cos(phi)*sin(psi), ...
          cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)];
         [cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta),  cos(phi)*cos(psi), ...
          sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)];
         [-cos(phi)*sin(theta), sin(phi), cos(phi)*cos(theta)]];
end

function eul = rotmat2eulzxy(R)
    if R(3,2) < 1
        if R(3,2) > -1
            thetaX = asin(R(3,2));
            thetaZ = atan2(-R(1,2), R(2,2));
            thetaY = atan2(-R(3,1), R(3,3));
        else % R(3,2) == -1
            thetaX = -pi/2;
            thetaZ = -atan2(R(1,3),R(1,1));
            thetaY = 0;
        end
    else % R(3,2) == +1
        thetaX = pi/2;
        thetaZ = atan2(R(1,3),R(1,1));
        thetaY = 0;
    end
    eul = [thetaX, thetaY, thetaZ];
end

function w = veemap(R)
    w = [-R(2,3), R(1,3), -R(1,2)];
end
