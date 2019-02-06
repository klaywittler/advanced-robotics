function [desired_state] = diamond(t, qn)
% DIAMOND trajectory generator for a diamond

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
% NOTE: the simulator will spawn the robot to be at the
%       position you return for t == 0

t_final = 12.5;

p1 = [0;0;0];
p2 = [1/4;sqrt(2);sqrt(2)];
p3 = [1/2;0;2*sqrt(2)];
p4 = [3/4;-sqrt(2);sqrt(2)];
p5 = [1;0;0];

d1 = norm(p2-p1);
d2 = norm(p3-p2);
d3 = norm(p4-p3);
d4 = norm(p5-p4);
d = d1 + d2 + d3 + d4;

t1 = d1*t_final/d;
t2 = d2*t_final/d + t1;
t3 = d3*t_final/d + t2;
t4 = d4*t_final/d + t3;

if t <= t1
    dt = t;
    tf = t1;
    pOld = p1;
    pNew = p2;
elseif t <= t2
    dt = t - t1;
    tf = t2 - t1;
    pOld = p2;
    pNew = p3;  
elseif t <= t3
    dt = t - t2;
    tf = t3-t2;
    pOld = p3;
    pNew = p4;
elseif t <= t4
    dt = t - t3;
    tf = t4-t3;
    pOld = p4;
    pNew = p5;
else
    dt = 0;
    tf = t4-t3;
    pOld = p5;
    pNew = p5;    
end

c = (pNew-pOld)/((1/2 - 1/3)*tf^3);
pos = -c*((1/3)*dt^3 - (tf/2)*dt^2) + pOld;
vel = -c*(dt^2 - tf*dt);
acc = -c*(2*dt - tf);
yaw = 0;
yawdot = 0;

% =================== Your code ends here ===================

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end
