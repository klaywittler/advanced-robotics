function [pA] = aprilTagMap()
%APRILTAG Summary of this function goes here
%   Detailed explanation goes here
pA = zeros(2,5,12*9);
pInit0 = [0.152/2, 0.152, 0.152, 0, 0; 0.152/2, 0, 0.152, 0.152, 0];
c = 1;
for j=1:9
    pInit = pInit0;
    for i=1:12
        pA(:,:,c) = pInit;
        pInit = pInit + 2*0.152*[ones(1,5);zeros(1,5)];
        c = c + 1;
    end 
    if j== 3 || j == 6
        pInit0 = pInit0 + 0.178*[zeros(1,5);ones(1,5)] + 0.152*[zeros(1,5);ones(1,5)]; 
    else
        pInit0 = pInit0 + 2*0.152*[zeros(1,5);ones(1,5)];
    end
end
end

