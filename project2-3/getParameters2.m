function [F,V,Xdot] = getParameters2(in1,in2,in3,dt)
%GETPARAMETERS2
%    [F,V,XDOT] = GETPARAMETERS2(IN1,IN2,IN3,DT)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    14-Apr-2019 10:25:14

ax = in2(1,:);
ay = in2(2,:);
az = in2(3,:);
bax = in1(13,:);
bay = in1(14,:);
baz = in1(15,:);
bgx = in1(10,:);
bgy = in1(11,:);
bgz = in1(12,:);
nax = in3(1,:);
nay = in3(2,:);
naz = in3(3,:);
nbax = in3(10,:);
nbay = in3(11,:);
nbaz = in3(12,:);
nbgx = in3(7,:);
nbgy = in3(8,:);
nbgz = in3(9,:);
ngx = in3(4,:);
ngy = in3(5,:);
ngz = in3(6,:);
phi = in1(4,:);
psi = in1(6,:);
theta = in1(5,:);
vx = in1(7,:);
vy = in1(8,:);
vz = in1(9,:);
wx = in2(4,:);
wy = in2(5,:);
wz = in2(6,:);
t2 = cos(theta);
t3 = sin(theta);
t4 = bgx.*t3;
t5 = t2.*wz;
t6 = ngx.*t3;
t12 = bgz.*t2;
t13 = ngz.*t2;
t14 = t3.*wx;
t7 = t4+t5+t6-t12-t13-t14;
t8 = cos(phi);
t9 = sin(phi);
t10 = 1.0./t8;
t11 = 1.0./t8.^2;
t15 = sin(psi);
t16 = -ax+bax+nax;
t17 = cos(psi);
t18 = -az+baz+naz;
t19 = -ay+bay+nay;
t20 = t3.*t17;
t21 = t2.*t9.*t15;
t22 = t20+t21;
t23 = t3.*t15;
t51 = t2.*t9.*t17;
t24 = t23-t51;
t25 = t2.*t15;
t26 = t3.*t9.*t17;
t27 = t25+t26;
t28 = t2.*t17;
t47 = t3.*t9.*t15;
t29 = t28-t47;
t30 = dt.*t2.*t9.*t10;
t31 = dt.*t3.*t10;
t32 = dt.*t3.*t9.*t15;
t33 = t32-dt.*t2.*t17;
t34 = dt.*t8.*t15;
t35 = dt.*t2.*t9.*t17;
t36 = t35-dt.*t3.*t15;
t37 = dt.*t3.*t8;
t38 = bgx.*t2;
t39 = ngx.*t2;
t40 = bgz.*t3;
t41 = ngz.*t3;
t42 = t38+t39+t40+t41-t2.*wx-t3.*wz;
t43 = t2.^2;
t44 = t3.^2;
t45 = t43+t44;
t46 = 1.0./t45;
t48 = t16.*t29;
t49 = t18.*t22;
t50 = t16.*t27;
t52 = t18.*t24;
t53 = t8.*t17.*t19;
F = reshape([1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,-dt.*t7.*t11,dt.*t9.*t11.*(t4+t5+t6-t12-t13-t14),-dt.*(t9.*t15.*t19-t3.*t8.*t15.*t16+t2.*t8.*t15.*t18),dt.*(t9.*t17.*t19-t3.*t8.*t16.*t17+t2.*t8.*t17.*t18),-dt.*(t8.*t19+t3.*t9.*t16-t2.*t9.*t18),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,dt.*t7,-t10.*(-t8+bgx.*dt.*t2.*t9+bgz.*dt.*t3.*t9+dt.*ngx.*t2.*t9+dt.*ngz.*t3.*t9-dt.*t2.*t9.*wx-dt.*t3.*t9.*wz),dt.*t10.*t42,dt.*(t16.*t22-t18.*t29),dt.*(t16.*t24-t18.*t27),dt.*(t2.*t8.*t16+t3.*t8.*t18),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,dt.*(t50+t52+t53),-dt.*(t48+t49-t8.*t15.*t19),0.0,0.0,0.0,0.0,0.0,0.0,0.0,dt,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,dt,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,dt,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-dt.*t2,-dt.*t3.*t9.*t10,t31,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-dt,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-dt.*t3,t30,-dt.*t2.*t10,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t33,-dt.*t27,t37,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t34,-dt.*t8.*t17,-dt.*t9,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-dt.*t22,t36,-dt.*t2.*t8,0.0,0.0,0.0,0.0,0.0,1.0],[15,15]);
if nargout > 1
    V = reshape([0.0,0.0,0.0,0.0,0.0,0.0,t33,-dt.*t27,t37,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t34,-dt.*t8.*t17,-dt.*t9,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-dt.*t22,t36,-dt.*t2.*t8,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-dt.*t2,-dt.*t3.*t9.*t10,t31,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-dt,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-dt.*t3,t30,-dt.*t2.*t10,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,dt,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,dt,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,dt,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,dt,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,dt,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,dt],[15,12]);
end
if nargout > 2
    Xdot = [vx;vy;vz;-t42.*t46;-t10.*t46.*(bgx.*t3.*t9+bgy.*t8.*t43+bgy.*t8.*t44-bgz.*t2.*t9+ngx.*t3.*t9+ngy.*t8.*t43+ngy.*t8.*t44-ngz.*t2.*t9-t3.*t9.*wx-t8.*t43.*wy-t8.*t44.*wy+t2.*t9.*wz);t10.*t46.*(t4+t5+t6-t12-t13-t14);-t48-t49+t8.*t15.*t19;-t50-t52-t53;-t9.*t19+t3.*t8.*t16-t2.*t8.*t18+1.0;nbgx;nbgy;nbgz;nbax;nbay;nbaz];
end
