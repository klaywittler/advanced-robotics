function [v,Aeq,beq,H] = minsnap(n,d,w,dt)
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

dim = numel(w(1,:));
Aeq = zeros(0,dim*d*n);
beq = zeros(0,1);
H = zeros(dim*d*n,dim*d*n);

% calculate correct values for Aeq, beq, and H

for i=1:n
    [Aeq_i, beq_i] = Ab_i1(i, n, d, dt(i), w(i,:)', w(i+1,:)');
    Aeq = [Aeq; Aeq_i];
    beq = [beq; beq_i];

end

for i=1:(n-1)
   for k=1:4
       [Aeq_i, beq_i] = Ab_i2(i, k, n, d, dt(i));
       Aeq = [Aeq; Aeq_i];
       beq = [beq; beq_i];
   end
end

for i=1:n
    H = H + 2*H_i1(i, n, d, dt(i));
end

% solves the quadratic program for v
v = quadprog(H,zeros(dim*d*n,1),zeros(0,dim*d*n),zeros(0,1),Aeq,beq);
end

