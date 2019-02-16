function [A_i2, b_i2] = Ab_i2(i, k, n, d, dt_i)
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

dim = 3;
A_i2 = zeros(dim,dim*d*n);
b_i2 = zeros(dim,1);
for j=k:(d-1)
   A_i2(1,1 + (i-1)*dim*d + dim*j) = (factorial(j)/factorial(j-k))*dt_i^(j-k);
   A_i2(2,2 + (i-1)*dim*d + dim*j) = (factorial(j)/factorial(j-k))*dt_i^(j-k);
   A_i2(3,3 + (i-1)*dim*d + dim*j) = (factorial(j)/factorial(j-k))*dt_i^(j-k);
end
A_i2(1,1 + (i)*dim*d + dim*k) = -factorial(k);
A_i2(2,2 + (i)*dim*d + dim*k) = -factorial(k);
A_i2(3,3 + (i)*dim*d + dim*k) = -factorial(k);

end

