function [F,V,Xdot] = getParameters1(x,u,n,dt)
%VECOTRMAP Summary of this function goes here
%   Detailed explanation goes here
phi = x(4);
theta = x(5);
bgx = x(7);

nvx = n(1);
nvy = n(2);
nvz = n(3);
ngx = n(4);
ngy = n(5);
ngz = n(6);
nbgx = n(7); 
nbgy = n(8);
nbgz = n(9);

vx = u(1);
vy = u(2);
vz = u(3);
wx = u(4);
wy = u(5);
wz = u(6);

[F,V,Xdot] = linearizedModel1(bgx,dt,nbgx,nbgy,nbgz,ngx,ngy,ngz,nvx,nvy,nvz,phi,theta,vx,vy,vz,wx,wy,wz);
end

