function [desired_state] = circle(t, qn)
% CIRCLE trajectory generator for a circle

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
% NOTE: the simulator will spawn the robot to be at the
%       position you return for t == 0

t_final = 13;

a = 2*pi/((1/6 - 1/4)*t_final^3);
b = -a*t_final/2;
theta = (a/6)*t^3 + (b/2)*t^2;
thetadot = (a/2)*t^2 + b*t;
thetaddot = a*t + b;
R = 5;

if t <= t_final
    pos = [R*cos(theta); R*sin(theta); theta*2.5/(2*pi)]; %
    vel = [-R*sin(theta)*thetadot; R*cos(theta)*thetadot; thetadot*2.5/(2*pi)]; %
    acc = [-R*sin(theta)*thetaddot-R*cos(theta)*thetadot^2; R*cos(theta)*thetaddot-R*sin(theta)*thetadot^2; 0]; %
    yaw = 0;
    yawdot = 0;
else
    pos = [5; 0; 2.5];
    vel = [0; 0; 0];
    acc = [0; 0; 0];
    yaw = 0;
    yawdot = 0;   
end

% =================== Your code ends here ===================

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end
