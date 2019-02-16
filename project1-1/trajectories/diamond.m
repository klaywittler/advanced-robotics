function [desired_state] = diamond(t, qn)
% DIAMOND trajectory generator for a diamond

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
% NOTE: the simulator will spawn the robot to be at the
%       position you return for t == 0

p = [0,0,0;
    1/4,sqrt(2),sqrt(2);
    1/2,0,2*sqrt(2)];
%     3/4,-sqrt(2),sqrt(2);
%     1,0,0;
%     0.5,0,0];

p2 = p(2:end,:);
p1 = p(1:end-1,:);

d = sqrt(sum((p2-p1).^2, 2));

% d = vecnorm(p2-p1,2,2);
cumDist = cumsum(d);
totalDist = sum(d);

maxVel = 0.6; % m/s
tFinal = totalDist/maxVel;

ratio = cumDist/totalDist;
tI = [0;ratio*tFinal];
dt = tI(2:end) - tI(1:end-1);
dim = 3;
n = numel(p(:,1))-1;
d = 14;

[v,Aeq,beq,H] = minsnap(n,d,p,dt);
c = zeros(3,d,n);
for i=1:n
    for j=0:d-1
        c(:,j+1,i) = v(1 + (i-1)*dim*d + dim*j:3 + (i-1)*dim*d + dim*j); 
    end
end
if t <= tI(end)
    dtI = t-tI; % dt = t - t1;
    j = find(dtI < 0, 1) - 1;
    if isempty(j)
        j = numel(dtI);
    end
    dt = t - tI(j);
    pow0 = 0:d-1; 
    pow1 = pow0 - 1;
    pow2 = pow1 - 1;
    pow1(pow1 < 0) = [];
    pow2(pow2 < 0) = [];
    pos = c(:,:,j)*(dt.^pow0)';
    vel = c(:,:,j)*([0 ,pow0(2:end).*(dt.^(pow1))])';
    acc = c(:,:,j)*([0, 0, pow0(3:end).*pow1(2:end).*(dt.^(pow2))])';
%     newP = j;
%     oldP = j-1;
% 
%     tf = tI(j)-tI(j-1); % tf = t2 - t1;
%     A = [tI(1)^7, tI(1)^6, tI(1)^5, tI(1)^4, tI(1)^3, tI(1)^2, tI(1), 1, zeros(1,8); % postion
%         tI(2)^7, tI(2)^6, tI(2)^5, tI(2)^4, tI(2)^3, tI(2)^2, tI(2), 1, zeros(1,8); 
%         zeros(1,8), tI(2)^7, tI(2)^6, tI(2)^5, tI(2)^4, tI(2)^3, tI(2)^2, tI(2), 1;
%         zeros(1,8), tI(3)^7, tI(3)^6, tI(3)^5, tI(3)^4, tI(3)^3, tI(3)^2, tI(3), 1;
%         7*tI(1)^6, 6*tI(1)^5, 5*tI(1)^4, 4*tI(1)^3, 3*tI(1)^2, 2*tI(1), 1, 0,zeros(1,8); % velocity
%         7*tI(2)^6, 6*tI(2)^5, 5*tI(2)^4, 4*tI(2)^3, 3*tI(2)^2, 2*tI(2), 1, 0, -7*tI(2)^6, -6*tI(2)^5, -5*tI(2)^4, -4*tI(2)^3, -3*tI(2)^2, -2*tI(2), -1, 0;
%         zeros(1,8), 7*tI(3)^6, 6*tI(3)^5, 5*tI(3)^4, 4*tI(3)^3, 3*tI(3)^2, 2*tI(3), 1, 0;
%         7*6*tI(1)^5, 6*5*tI(1)^4, 5*4*tI(1)^3, 4*3*tI(1)^2, 3*2*tI(1), 2, 0, 0, zeros(1,8); % accleration
%         7*6*tI(2)^5, 6*5*tI(2)^4, 5*4*tI(2)^3, 4*3*tI(2)^2, 3*2*tI(2), 2, 0, 0, -7*6*tI(2)^5, -6*5*tI(2)^4, -5*4*tI(2)^3, -4*3*tI(2)^2, -3*2*tI(2), -2, 0, 0;
%         zeros(1,8), 7*6*tI(3)^5, 6*5*tI(3)^4, 5*4*tI(3)^3, 4*3*tI(3)^2, 3*2*tI(3), 2, 0, 0;
%         7*6*5*tI(1)^4, 6*5*4*tI(1)^3, 5*4*3*tI(1)^2, 4*3*2*tI(1), 3*2, 0, 0, 0, zeros(1,8); % jerk
%         7*6*5*tI(2)^4, 6*5*4*tI(2)^3, 5*4*3*tI(2)^2, 4*3*2*tI(2), 3*2, 0, 0, 0, -7*6*5*tI(2)^4, -6*5*4*tI(2)^3, -5*4*3*tI(2)^2, -4*3*2*tI(2), -3*2, 0, 0, 0;
%         zeros(1,8), 7*6*5*tI(3)^4, 6*5*4*tI(3)^3, 5*4*3*tI(3)^2, 4*3*2*tI(3), 3*2, 0, 0, 0;
%         7*6*5*4*tI(1)^3, 6*5*4*3*tI(1)^2, 5*4*3*2*tI(1), 4*3*2, 0, 0, 0, 0, zeros(1,8); % snap
%         7*6*5*4*tI(2)^3, 6*5*4*3*tI(2)^2, 5*4*3*2*tI(2), 4*3*2, 0, 0, 0, 0, -7*6*5*4*tI(2)^3, -6*5*4*3*tI(2)^2, -5*4*3*2*tI(2), -4*3*2, 0, 0, 0, 0;
%         zeros(1,8), 7*6*5*4*tI(3)^3, 6*5*4*3*tI(3)^2, 5*4*3*2*tI(3), 4*3*2, 0, 0, 0, 0;
%         ];
%               
%     b = [p(1,:);p(2,:);p(2,:);p(3,:); zeros(12,3)];
%     x = A\b;
%     c0 = x(1, :);
%     c1 = x(2, :);
%     c2 = x(3, :);
%     c3 = x(4, :);
%     c4 = x(5, :);
%     c5 = x(6, :);
%     pos = [t^7, t^6, t^5, t^4, t^3, t^2, t, 1]*x((j-2)*8+1:(j-1)*8,:);
%     vel = [7*t^6, 6*t^5, 5*t^4, 4*t^3, 3*t^2, 2*t, 1, 0]*x((j-2)*8+1:(j-1)*8,:);
%     acc = [7*6*t^5, 6*5*t^4, 5*4*t^3, 4*3*t^2, 3*2*t, 2, 0, 0]*x((j-2)*8+1:(j-1)*8,:);
else
    pos = p(end,:);
    vel = zeros(3,1);
    acc = zeros(3,1);
end

yaw = 0;
yawdot = 0;

% =================== Your code ends here ===================

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end
