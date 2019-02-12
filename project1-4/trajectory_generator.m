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
end
 
p2 = p(2:end,:);
p1 = p(1:end-1,:);
d = vecnorm(p2-p1,2,2);
cumDist = cumsum(d);
totalDist = sum(d);

maxVel = 1; % m/s
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
    c = (p(newP,:)-p(oldP,:))/((1/2 - 1/3)*tf^3);
    pos = -c*((1/3)*dt(oldP)^3 - (tf/2)*dt(oldP)^2) + p(oldP,:);
    vel = -c*(dt(oldP)^2 - tf*dt(oldP));
    acc = -c*(2*dt(oldP) - tf);
else
    pos = p(end,:);
    vel = zeros(3,1);
    acc = zeros(3,1);    
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
