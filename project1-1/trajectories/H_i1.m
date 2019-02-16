function [H_i] = H_i1(i, n, d, dt_i)
%H_i1(i, n, d, dt_i, w_i, w_ip1) computes the integral of snap squared over
% the ith polynomial.
%   @param i - index of the polynomial.
%   @param n - total number of polynomials.
%   @param d - number of terms in each polynomial.
%   @param dt_i - \Delta t_i, duration of the ith polynomial.
%
%   @output H_i - matrix such that the snap squared integral is equal to
%   v^T H_i v

dim = 3;
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

