function [A_i1, b_i1] = Ab_i1(i, n, d, dt_i, w_i, w_ip1)
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

dim = numel(w_i);
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

