function [F,V,Xdot] = getParameters2(x,u,n,dt)
%VECOTRMAP Summary of this function goes here
%   Detailed explanation goes here
phi = x(4);
theta = x(5);
psi = x(6);
vx = x(7);
vy = x(8);
vz = x(9);
bgx = x(10);
bgy = x(11);
bgz = x(12);
bax = x(13);
bay = x(14);
baz = x(15);

nax = n(1);
nay = n(2);
naz = n(3);
ngx = n(4);
ngy = n(5);
ngz = n(6);
nbgx = n(7); 
nbgy = n(8);
nbgz = n(9);
nbax = n(10);
nbay = n(11);
nbaz = n(12);

ax = u(1);
ay = u(2);
az = u(3);
wx = u(4);
wy = u(5);
wz = u(6);

[F,V,Xdot] = linearizedModel2(ax,ay,az,bax,bay,baz,bgx,bgy,bgz,dt,nax,nay,naz,nbax,nbay,nbaz,nbgx,nbgy,nbgz,ngx,ngy,ngz,phi,psi,theta,vx,vy,vz,wx,wy,wz);
end

