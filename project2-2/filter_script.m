close all; clear all;
load('data/studentdata1.mat');
t = [data.t];
dt = t(2:end) - t(1:end-1);
f = zeros(size(dt));

s = 0.0205*ones(4,1);
sf = 0.0205*ones(4,1);

% [b,a] = cheby1(3,5,0.4); 
[b,a] = butter(3,0.2); 
%b = [0.25,0.25,0.25,0.25];
% a= b; 
%f = filter(b,a,dt,s);

for i=1:numel(dt)
   s = [s(2); s(3); s(4); dt(i)];
   f(i) = a*s + b*sf;
   sf = [sf(2); sf(3); sf(4); f(i)];
end
    
s = 0.0205*ones(numel(dt)+1,1);
alpha = 0.2;
stm1 = 0.0205;
for i=1:numel(dt)
    s(i+1) = alpha*dt(i) + (1-alpha)*s(i);
end

plot(f)