function [pA] = aprilTagMap()
%APRILTAG returns world coordinates for an April Tag map
%   Inputs:
%       nX: number of April Tags along the x direction
%       nY: number of April Tags along the y direction
%       pInit0: 2xn vector containing positions of the first April Tag
%       l: side length in meters for April Tag (assuming square)
%       s: spacing in meters between April Tags
%   Outputs:
%       pA : 3D vector representing April Tag coordines in world frame 
%            with convention of (x,y) position in first dimension, 
%            (p0,p1,..,,pn-1) in second dimension, and (id) in 3 dimension
nX = 12;
nY = 9;
l = 0.152;
pInit0 = [0.152/2, 0.152, 0.152, 0, 0; 0.152/2, 0, 0.152, 0.152, 0];
n = numel(pInit0(1,:));
pA = zeros(2,n,nX*nY);
c = 1;
for j=1:nY
    pInit = pInit0;
    for i=1:nX
        pA(:,:,c) = pInit;
        pInit = pInit + 2*l*[ones(1,n);zeros(1,n)];
        c = c + 1;
    end 
    if j== 3 || j == 6
        pInit0 = pInit0 + 0.178*[zeros(1,n);ones(1,n)] + l*[zeros(1,n);ones(1,n)]; 
    else
        pInit0 = pInit0 + 2*l*[zeros(1,n);ones(1,n)];
    end
end
end

