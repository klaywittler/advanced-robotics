syms x y z theta phi psi bgx bgy bgz vx vy vz wx wy wz ngx ngy ngz nvx nvy nvz nbgx nbgy nbgz dt

X = [x;y;z;phi;theta;psi;bgx;bgy;bgz]; 
U = [vx;vy;vz;wx;wy;wz];
N = [nvx;nvy;nvz;ngx;ngy;ngz;nbgx;nbgy;nbgz];

G = [cos(X(5)) 0 -cos(X(4))*sin(X(5));
        0 1 sin(X(4));
        sin(X(5)) 0 cos(X(4))*cos(X(5))];
    
Xdot = [U(1:3) - N(1:3);
        inv(G)*(U(4:6) - X(7) - N(4:6));
        N(7:9)];
    
F = simplify(jacobian(Xdot,X)*dt + eye(9));
V = simplify(jacobian(Xdot,N));
C = [eye(3) zeros(3) zeros(3);
    zeros(3) eye(3) zeros(3)];

matlabFunction(F,V,Xdot,'File','getParameters1','Vars',{[x;y;z;phi;theta;psi;bgx;bgy;bgz],[vx;vy;vz;wx;wy;wz],[nvx;nvy;nvz;ngx;ngy;ngz;nbgx;nbgy;nbgz],dt});
% matlabFunction(Xdot,'File','motionModel1');
% U = matlabFunction(V);
