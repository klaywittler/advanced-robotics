close all; clear all;
addpath('submission')

syms x y z theta phi psi vx vy vz bgx bgy bgz bax bay baz ax ay az wx wy wz ngx ngy ngz nax nay naz nbgx nbgy nbgz nbax nbay nbaz dt

X = [x;y;z;phi;theta;psi;vx;vy;vz;bgx;bgy;bgz;bax;bay;baz]; 
U = [ax;ay;az;wx;wy;wz];
N = [nax;nay;naz;ngx;ngy;ngz;nbgx;nbgy;nbgz;nbax;nbay;nbaz];
g = [0;0;1];
% g = [0;0;-9.81];
G = [cos(X(5)) 0 -cos(X(4))*sin(X(5));
        0 1 sin(X(4));
        sin(X(5)) 0 cos(X(4))*cos(X(5))];
R = eulzxy2rot([X(4);X(5);X(6)]);
    
Xdot = [X(7:9);
        G\(U(4:6) - X(10:12) - N(4:6));
        g + R*(U(1:3) - X(13:15) - N(1:3));
        N(7:9);
        N(10:12)];
    
F = simplify(jacobian(Xdot,X)*dt + eye(15));
V = simplify(jacobian(Xdot,N)*dt);
C = [eye(3) zeros(3) zeros(3) zeros(3);
    zeros(3) eye(3) zeros(3) zeros(3);
    zeros(3) zeros(3) eye(3) zeros(3)];

matlabFunction(F,V,Xdot,'File','getParameters2','Vars',{[x;y;z;phi;theta;psi;vx;vy;vz;bgx;bgy;bgz;bax;bay;baz],[ax;ay;az;wx;wy;wz],[nax;nay;naz;ngx;ngy;ngz;nbgx;nbgy;nbgz;nbax;nbay;nbaz],dt});