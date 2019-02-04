function [desired_state] = circle(t, qn)
% CIRCLE trajectory generator for a circle

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
% NOTE: the simulator will spawn the robot to be at the
%       position you return for t == 0

t_final = 20;
theta = 2*pi*t/t_final;
thetadot = 2*pi/t_final;
thetaddot = 0;
R = 5;
c = 2.5/((1/2 - 1/3)*t_final^3);

if t <= t_final
    pos = [R*cos(theta); R*sin(theta); -c*((1/3)*t^3 - (t_final/2)*t^2)]; %
    vel = [-R*sin(theta)*thetadot; R*cos(theta)*thetadot; -c*(t^2 - t_final*t)]; %
    acc = [-R*sin(theta)*thetaddot-R*cos(theta)*thetadot^2; R*cos(theta)*thetaddot-R*sin(theta)*thetadot^2; -c*(2*t - t_final)]; %
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
