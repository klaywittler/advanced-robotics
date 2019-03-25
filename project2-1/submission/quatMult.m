function [q] = quatMult(q1,q2)
%QUATMULT Summary of this function goes here
%   Detailed explanation goes here
    qw = q2(1)*q1(1) - q2(2)*q1(2) - q2(3)*q1(3) - q2(4)*q1(4);
    qi = q2(1)*q1(2) + q2(2)*q1(1) - q2(3)*q1(4) + q2(4)*q1(3);
    qj = q2(1)*q1(3) + q2(2)*q1(4) + q2(3)*q1(1) - q2(4)*q1(2);
    qk = q2(1)*q1(4) - q2(2)*q1(3) + q2(3)*q1(2) + q2(4)*q1(1);
    q = [qw;qi;qj;qk];
end

