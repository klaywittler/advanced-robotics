function [desired_state] = trajectory_generator(t, qn, varargin)
% TRAJECTORY_GENERATOR: Turn a Dijkstra or A* path into a trajectory
% t: A scalar, specifying inquiry time
%
% varargin: variable number of input arguments. In the framework,
% this function will first (and only once!) be called like this:
%
% trajectory_generator([],[], 0, path)
%
% i.e. map = varargin{1} and path = varargin{2}.
%
% path: A N x 3 matrix where each row is (x, y, z) coordinate of a
% point in the path. N is the total number of points in the path
%
% This is when you compute and store the trajectory.
%
% Later it will be called with only t and qn as an argument, at
% which point you generate the desired state for point t.
%
% use the "persistent" keyword to keep your trajectory around
% inbetween function calls

persistent p  
if ~isempty(varargin)
    p = varargin{2};
    pos = zeros(3,1);
    vel = zeros(3,1);
    acc = zeros(3,1);   
else
    p2 = p(2:end,:);
    p1 = p(1:end-1,:);
    d = sqrt(sum((p2-p1).^2, 2));
    % d = vecnorm(p2-p1,2,2);
    cumDist = cumsum(d);
    totalDist = sum(d);

    maxVel = 0.4; % m/s
    tFinal = totalDist/maxVel;

    ratio = cumDist/totalDist;
    tInterval = [0;ratio*tFinal];

    if t <= tInterval(end)
        dt = t-tInterval; % dt = t - t1;
        j = find(dt < 0, 1);
        if isempty(j)
            j = numel(dt);
        end
        newP = j;
        oldP = j-1;

        tf = tInterval(j)-tInterval(j-1); % tf = t2 - t1; 

    %     c = (p(newP,:)-p(oldP,:))/((1/2 - 1/3)*tf^3);
    %     pos = -c*((1/3)*dt(oldP)^3 - (tf/2)*dt(oldP)^2) + p(oldP,:);
    %     vel = -c*(dt(oldP)^2 - tf*dt(oldP));
    %     acc = -c*(2*dt(oldP) - tf);

            q0 = p(oldP,:);
            v0 = zeros(1,3);
            a0 = zeros(1,3);
            j0 = zeros(1,3);
            qf = p(newP,:);
            vf = zeros(1,3);
            af = zeros(1,3);
            jf = zeros(1,3);
            A = [1, 0, 0, 0, 0, 0, 0, 0;
                0, 1, 0, 0, 0, 0, 0, 0;
                0, 0, 2, 0, 0, 0, 0, 0;
                0, 0, 0, 6, 0, 0, 0, 0;
                1, tf, tf^2, tf^3, tf^4, tf^5, tf^6, tf^7;
                0, 1, 2*tf, 3*tf^2, 4*tf^3, 5*tf^4, 6*tf^5, 7*tf^6;
                0, 0, 2, 6*tf, 12*tf^2, 20*tf^3, 30*tf^4, 42*tf^5;
                0, 0, 0, 6, 24*tf, 60*tf^2, 120*tf^3, 210*tf^4];
            b = [q0; v0; a0; j0; qf; vf; af; jf];
            x = A\b;
            c0 = x(1, :);
            c1 = x(2, :);
            c2 = x(3, :);
            c3 = x(4, :);
            c4 = x(5, :);
            c5 = x(6, :);
            c6 = x(7, :);
            c7 = x(8, :);
            pos = c0 + c1*dt(oldP) + c2*dt(oldP)^2 + c3*dt(oldP)^3 + c4*dt(oldP)^4 + c5*dt(oldP)^5 + c6*dt(oldP)^6 + c7*dt(oldP)^7;
            vel = c1 + 2*c2*dt(oldP) + 3*c3*dt(oldP)^2 + 4*c4*dt(oldP)^3 + 5*c5*dt(oldP)^4 + 6*c6*dt(oldP)^5 + 7*c7*dt(oldP)^6;
            acc = 2*c2 + 6*c3*dt(oldP) + 12*c4*dt(oldP)^2 + 20*c5*dt(oldP)^3 + 30*c6*dt(oldP)^4 + 42*c7*dt(oldP)^5;
    else
        pos = p(end,:);
        vel = zeros(3,1);
        acc = zeros(3,1);    
    end

end
yaw = 0;
yawdot = 0;

%
% When called without varargin (isempty(varargin) == true), compute
% and return the desired state here.
%

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;
