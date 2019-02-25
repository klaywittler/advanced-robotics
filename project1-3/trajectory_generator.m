function [ desired_state ] = trajectory_generator(t, qn, map, path)
% TRAJECTORY_GENERATOR: Turn a Dijkstra or A* path into a trajectory
%
% NOTE: This function would be called with variable number of input
% arguments. At first, it will be called with arguments
% trajectory_generator([], [], map, path) and later, in test_trajectory,
% it will be called with only t and qn as arguments, so your code should
% be able to handle that. This can be done by checking the number of
% arguments to the function using the "nargin" variable, check the
% MATLAB documentation for more information.
%
% parameters:
%   map: The map structure returned by the load_map function
%   path: This is the path returned by your planner (dijkstra function)
%   desired_state: Contains all the information that is passed to the
%                  controller, as in Phase 1
%
% NOTE: It is suggested to use "persistent" variables to store map and path
% during the initialization call of trajectory_generator, e.g.
% persistent map0 path0
% map0 = map;
% path0 = path;

persistent p tI c
deg = 8; % degree of polynomial
avgAccel = 0.28; % m/s
if nargin > 2
    desired_state = [];
    p = path;
    p2 = p(2:end,:);
    p1 = p(1:end-1,:);
    d = sqrt(sum((p2-p1).^2, 2));
    cumDist = cumsum(d);
    totalDist = sum(d);
    tFinal = sqrt(2*totalDist/avgAccel);
    ratio = cumDist/totalDist;
    tI = [0;ratio*tFinal];
    dt = tI(2:end) - tI(1:end-1);
    dim =  numel(p(1,:));
    n = numel(p(:,1))-1;
    v = minsnap2(n,deg,p,dt,dim);
    c = reshape(v,[3,deg,n]);
     ploting(c,tI,tFinal,deg);
else
    if t <= tI(end)
       dtI = t-tI; % dt = t - t1;
        j = find(dtI < 0, 1) - 1;
        if isempty(j)
            j = numel(dtI);
        end
        dt = t - tI(j);
        pow0 = 0:deg-1; 
        pow1 = pow0 - 1;
        pow2 = pow1 - 1;
        pow1(pow1 < 0) = [];
        pow2(pow2 < 0) = [];
        pos = c(:,:,j)*(dt.^pow0)';
        vel = c(:,:,j)*([0 ,pow0(2:end).*(dt.^(pow1))])';
        acc = c(:,:,j)*([0, 0, pow0(3:end).*pow1(2:end).*(dt.^(pow2))])';
    else
       pos = p(end,:);
       vel = zeros(3,1);
       acc = zeros(3,1);    
    end
    yaw = 0;
    yawdot = 0;

    desired_state.pos = pos(:);
    desired_state.vel = vel(:);
    desired_state.acc = acc(:);
    desired_state.yaw = yaw;
    desired_state.yawdot = yawdot;    
end

end
%%%%%%%%%%%%%%%%%%%%%%%%
%%% helper functions %%%
%%%%%%%%%%%%%%%%%%%%%%%%
function [v] = minsnap(n,d,w,dt,dim)
%MINSNAP(n,d,w,r,dt) calculates the polynomial coefficients for the minimum
%snap trajectory intersecting waypoints w.
%   @param n - total number of polynomials.
%   @param d - number of terms in each polynomial.
%   @param w - cell array of waypoints, containing w_i in w{i}.
%   @param dt - cell array of delays, containing \Delta t_i in dt{i}.
%
%   @output v - vector of polynomial coefficients, output of quadprog.
%   @output Aeq - A matrix from linear equality constraint A_eq v = b_eq
%   @output beq - b vector from linear equality constraint A_eq v = b_eq
%   @output H - matrix such that the integral of snap squared is .5 v^T H v

ddt = d-1;
Aeq = zeros(ddt*dim*(n+1) + dim*2*(n),dim*d*n);
beq = zeros(ddt*dim*(n+1) + dim*2*(n),1);
% H = zeros(dim*d*n,dim*d*n);

% calculate correct values for Aeq, beq, and H
j = 2*dim*n + 1;
for i=1:n
    [Aeq_i, beq_i] = Ab_iPosition(i, n, d, dt(i), w(i,:)', w(i+1,:)', dim);
    Aeq(2*dim*(i-1)+1:2*dim*i,:) = Aeq_i; 
    beq(2*dim*(i-1)+1:2*dim*i,:) = beq_i;
%     H = H + 2*H_i(i, n, d, dt(i), dim);
    if  i<n
        % matching derivatives between polynomials
        for k=1:ddt
            [Aeq_i, beq_i] = Ab_iDerivative(i, k, n, d, dt(i),dim);
            Aeq(dim*(k-1)+ j:dim*k + j-1,:) = Aeq_i; 
            beq(dim*(k-1)+ j:dim*k + j-1,:) = beq_i;
        end
        j = j + dim*ddt;
    else
        % constraint for 0 derivatives at beginning
        for k=1:ddt
            [Aeq_i, beq_i] = Ab_iDerivative(1, k, n, d, 0,dim);
            Aeq(dim*(k-1)+ j:dim*k + j-1,:) = Aeq_i; 
            beq(dim*(k-1)+ j:dim*k + j-1,:) = beq_i;
        end 
        j = j + dim*ddt;
        % constraint for 0 derivatives at end
        for k=1:ddt
            [Aeq_i, beq_i] = Ab_iDerivative(n, k, n, d, 0, dim);
            Aeq(dim*(k-1)+ j:dim*k + j-1,:) = Aeq_i; 
            beq(dim*(k-1)+ j:dim*k + j-1,:) = beq_i;
        end 
        j = j + dim*ddt;
    end
end
% solves the quadratic program for v
% v = quadprog(H,zeros(dim*d*n,1),zeros(0,dim*d*n),zeros(0,1),Aeq,beq);
v = Aeq\beq;
end

function [v] = minsnap2(n,d,w,dt,dim)
%MINSNAP(n,d,w,r,dt) calculates the polynomial coefficients for the minimum
%snap trajectory intersecting waypoints w.
%   @param n - total number of polynomials.
%   @param d - number of terms in each polynomial.
%   @param w - cell array of waypoints, containing w_i in w{i}.
%   @param dt - cell array of delays, containing \Delta t_i in dt{i}.
%
%   @output v - vector of polynomial coefficients, output of quadprog.
%   @output Aeq - A matrix from linear equality constraint A_eq v = b_eq
%   @output beq - b vector from linear equality constraint A_eq v = b_eq
%   @output H - matrix such that the integral of snap squared is .5 v^T H v

ddt = d-1;
Aeq = zeros(ddt*dim*(n) + dim*2*(n) + (floor(ddt/2)-2)*dim,dim*d*n);
beq = zeros(ddt*dim*(n) + dim*2*(n) + (floor(ddt/2)-2)*dim,1);
H = zeros(dim*d*n,dim*d*n);

% calculate correct values for Aeq, beq, and H
j = 2*dim*n + 1;
for i=1:n
    [Aeq_i, beq_i] = Ab_iPosition(i, n, d, dt(i), w(i,:)', w(i+1,:)', dim);
    Aeq(2*dim*(i-1)+1:2*dim*i,:) = Aeq_i; 
    beq(2*dim*(i-1)+1:2*dim*i,:) = beq_i;
    H = H + 2*H_i(i, n, d, dt(i), dim);
    if  i<n
        % matching derivatives between polynomials
        for k=1:ddt
            [Aeq_i, beq_i] = Ab_iDerivative(i, k, n, d, dt(i),dim);
            Aeq(dim*(k-1)+ j:dim*k + j-1,:) = Aeq_i; 
            beq(dim*(k-1)+ j:dim*k + j-1,:) = beq_i;
        end
        j = j + dim*ddt;
    else
        % constraint for 0 derivatives at beginning
        for k=1:ceil(ddt/2)
            [Aeq_i, beq_i] = Ab_iDerivative(1, k, n, d, 0,dim);
            Aeq(dim*(k-1)+ j:dim*k + j-1,:) = Aeq_i; 
            beq(dim*(k-1)+ j:dim*k + j-1,:) = beq_i;
        end 
        j = j + dim*ceil(ddt/2);
        % constraint for 0 derivatives at end
        for k=1:ceil(ddt/2)
            [Aeq_i, beq_i] = Ab_iDerivative(n, k, n, d, 0, dim);
            Aeq(dim*(k-1)+ j:dim*k + j-1,:) = Aeq_i; 
            beq(dim*(k-1)+ j:dim*k + j-1,:) = beq_i;
        end 
        j = j + dim*ceil(ddt/2);
    end
end
% solves the quadratic program for v
% v = quadprog(H,zeros(dim*d*n,1),zeros(0,dim*d*n),zeros(0,1),Aeq,beq);
v = Aeq\beq;
end

function [A_i1, b_i1] = Ab_iPosition(i, n, d, dt_i, w_i, w_ip1, dim)
%AB_I1(i, n, d, dt_i, w_i, w_ip1) computes the linear equality constraint
% constants to require the ith polynomial to meet waypoints w_i and w_{i+1}
% at it's endpoints.
%   @param i - index of the polynomial.
%   @param n - total number of polynomials.
%   @param d - number of terms in each polynomial.
%   @param dt_i - \Delta t_i, duration of the ith polynomial.
%   @param w_i - waypoint at the start of the ith polynomial.
%   @param w_ip1 - w_{i+1}, waypoint at the end of the ith polynomial.
%
%   @output A_i1 - A matrix from linear equality constraint A_i1 v = b_i1
%   @output b_i1 - b vector from linear equality constraint A_i1 v = b_i1

A_i1 = zeros(2*dim,dim*d*n);
% b_i1 = zeros(2*dim,1);
% coefficient for x_{i,0}
A_i1(1,1 + (i-1)*dim*d) = 1;
% coefficient for y_{i,0}
A_i1(2,2 + (i-1)*dim*d) = 1;
% coefficient for z_{i,0}
A_i1(3,3 + (i-1)*dim*d) = 1;
for j=0:(d-1)
    % coefficient for x_{i,j}
    A_i1(4,1 + (i-1)*dim*d + dim*j) = dt_i^j;
    % coefficient for y_{i,j}
    A_i1(5,2 + (i-1)*dim*d + dim*j) = dt_i^j;
    % coefficient for z_{i,j}
    A_i1(6,3 + (i-1)*dim*d + dim*j) = dt_i^j;
end

b_i1 = [w_i; w_ip1];

end

function [A_i2, b_i2] = Ab_iDerivative(i, k, n, d, dt_i, dim)
%AB_1(i, n, d, dt_i, w_i, w_ip1) computes the linear equality constraint
% constants to require the kth derivatives of the ith and (i+1)th
% polynomials to equal where they meet.
%   @param i - index of the polynomial.
%   @param k - order of derivative being taken.
%   @param n - total number of polynomials.
%   @param d - number of terms in each polynomial.
%   @param dt_i - \Delta t_i, duration of the ith polynomial.
%
%   @output A_i2 - A matrix from linear equality constraint A_i2 v = b_i2
%   @output b_i2 - b vector from linear equality constraint A_i2 v = b_i2

A_i2 = zeros(dim,dim*d*n);
b_i2 = zeros(dim,1);
for j=k:(d-1)
    A_i2(1,1 + (i-1)*dim*d + dim*j) = (factorial(j)/factorial(j-k))*dt_i^(j-k);
    A_i2(2,2 + (i-1)*dim*d + dim*j) = (factorial(j)/factorial(j-k))*dt_i^(j-k);
    A_i2(3,3 + (i-1)*dim*d + dim*j) = (factorial(j)/factorial(j-k))*dt_i^(j-k);
end
if dt_i ~= 0 
    A_i2(1,1 + (i)*dim*d + dim*k) = -factorial(k);
    A_i2(2,2 + (i)*dim*d + dim*k) = -factorial(k);
    A_i2(3,3 + (i)*dim*d + dim*k) = -factorial(k);
end
end

function [H_i] = H_i(i, n, d, dt_i, dim)
%H_i1(i, n, d, dt_i, w_i, w_ip1) computes the integral of snap squared over
% the ith polynomial.
%   @param i - index of the polynomial.
%   @param n - total number of polynomials.
%   @param d - number of terms in each polynomial.
%   @param dt_i - \Delta t_i, duration of the ith polynomial.
%
%   @output H_i - matrix such that the snap squared integral is equal to
%   v^T H_i v

H_i = zeros(dim*d*n,dim*d*n);

for k=4:(d-1)
    for l=4:(d-1)
        ex = k+l-7;
        H_i((i-1)*dim*d + dim*k + (1:dim),(i-1)*dim*d + dim*l + (1:dim)) = ...
            eye(dim)*(factorial(k)/factorial(k-4))...
            *(factorial(l)/factorial(l-4))*(dt_i^ex)/(ex);
    end
end

end

function ploting(c,tI, tF, deg)  
    pow0 = 0:deg-1; 
    pow1 = pow0 - 1;
    pow2 = pow1 - 1;
    pow3 = pow2 -1;
    pow4 = pow3 - 1;
    pow5 = pow4 - 1;
    pow6 = pow5 - 1;
    pow7 = pow6 - 1;
    pow1(pow1 < 0) = [];
    pow2(pow2 < 0) = [];
    pow3(pow3 < 0) = [];
    pow4(pow4 < 0) = [];
    pow5(pow5 < 0) = [];
    pow6(pow6 < 0) = [];
    pow7(pow7 < 0) = [];
    
    t = linspace(0, tF, 100);
    pos = zeros(3,numel(t)-1);
    vel = zeros(3,numel(t)-1);
    acc = zeros(3,numel(t)-1);
    jerk = zeros(3,numel(t)-1);
    snap = zeros(3,numel(t)-1);
    crackle = zeros(3,numel(t)-1);
    pop = zeros(3,numel(t)-1);
    sev = zeros(3,numel(t)-1);

    
    for i=1:numel(t)-1
        dtI = t(i)-tI; % dt = t - t1;
        j = find(dtI < 0, 1) - 1;
        if isempty(j)
           j = numel(dtI);
         end
         dt = t(i) - tI(j);
        pos(:,i) = c(:,:,j)*(dt.^pow0)';
        vel(:,i) = c(:,:,j)*([0 ,pow0(2:end).*(dt.^(pow1))])';
        acc(:,i) = c(:,:,j)*([0, 0, pow0(3:end).*pow1(2:end).*(dt.^(pow2))])';
        jerk(:,i) = c(:,:,j)*([0, 0, 0, pow0(4:end).*pow1(3:end).*pow2(2:end).*(dt.^(pow3))])';
        snap(:,i) = c(:,:,j)*([0, 0, 0, 0, pow0(5:end).*pow1(4:end).*pow2(3:end).*pow3(2:end).*(dt.^(pow4))])';
        crackle(:,i) = c(:,:,j)*([0, 0, 0, 0, 0, pow0(6:end).*pow1(5:end).*pow2(4:end).*pow3(3:end).*pow4(2:end).*(dt.^(pow5))])';
        pop(:,i) = c(:,:,j)*([0, 0, 0, 0, 0, 0, pow0(7:end).*pow1(6:end).*pow2(5:end).*pow3(4:end).*pow4(3:end).*pow5(2:end).*(dt.^(pow6))])';
        sev(:,i) = c(:,:,j)*([0, 0, 0, 0, 0, 0, 0, pow0(8:end).*pow1(7:end).*pow2(6:end).*pow3(5:end).*pow4(4:end).*pow5(3:end).*pow6(2:end).*(dt.^(pow7))])';
    end
    t = t(1:end-1);
end
