function [desired_state] = circle(t, qn)
% CIRCLE trajectory generator for a circle

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
% NOTE: the simulator will spawn the robot to be at the
%       position you return for t == 0

t_final = 11.9;

theta = 2*pi*t/t_final;
thetadot = 2*pi/t_final;
thetaddot = 0;
R = 5;
c = 2.5/((1/2 - 1/3)*t_final^3);
tb = 0.2;

if t <= t_final-tb
    pos = [R*cos(theta); R*sin(theta); theta*2.5/(2*pi)]; %
    vel = [-R*sin(theta)*thetadot; R*cos(theta)*thetadot; thetadot*2.5/(2*pi)]; %
    acc = [-R*sin(theta)*thetaddot-R*cos(theta)*thetadot^2; R*cos(theta)*thetaddot-R*sin(theta)*thetadot^2; 0]; %
    yaw = 0;
    yawdot = 0;
elseif t <= t_final
    dt = t - tb;
    tf = tb;
    c4 = 2.5*tb/t_final;
    c3 = 2.5/t_final;
    c2 = 0;
    c1 = (2.5 -c3*dt - c4)/dt^3;
    pOld = [R*cos(theta); R*sin(theta); theta*2.5/(2*pi)];
    pNew = [5; 0; 2.5];
    c = (pNew-pOld)/((1/2 - 1/3)*tf^3);
    pos = -c*((1/3)*dt^3 - (tf/2)*dt^2) + pOld;
    vel = -c*(dt^2 - tf*dt);
    acc = -c*(2*dt - tf);
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
