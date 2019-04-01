function [v, bestInliers] = motionRANSAC(p, dp, R, T)
% Estimate velocity given a set of 
% pairs of matching *calibrated* points
% X1,X2: Nx2 matrices of calibrated points
%   i^th row of X1 matches i^th row of X2
%
% E: robustly estimated E matrix
% bestInliers: indices of the rows of X1 (and X2) that where in the
% largest consensus set
sampleSize = 3;
eps = 10^(-4);
iter = 300;
bestNInliers = 0;
A = getA(p,R,T);

for i=1:iter
    indices = randperm(size(p,1));
    sampleInd = indices(1:sampleSize);
    testInd =  indices(sampleSize+1:length(indices));
    Asample = reshape(permute(A(sampleInd,:,:),[2,1,3]),[sampleSize*numel(p(1,:)),6]);
    Atest = reshape(permute(A(testInd,:,:),[2,1,3]),[(numel(p(:,1))-sampleSize)*numel(p(1,:)),6]);
    
    v_sample = estimate_velocity(dp(sampleInd,:),Asample);
    
    residuals = error(dp(testInd,:),v_sample,Atest); 
    
    curInliers = [testInd(residuals < eps), sampleInd];            % don't forget to include the sampleInd
    
    curNInliers = length(curInliers);

    if curNInliers > bestNInliers
        bestNInliers = curNInliers;
        bestInliers = curInliers;     
    end
end

Abest = reshape(permute(A(bestInliers,:,:),[2,1,3]),[bestNInliers*numel(p(1,:)),6]);
v = estimate_velocity(dp(bestInliers,:),Abest);
% disp(['Best number of inliers: ', num2str(bestNInliers), '/', num2str(size(p,1))]); 

end

function A = getA(p,R,T)
    A = zeros(numel(p(:,1)),2,6);
    for i=1:numel(p(:,1))
        z = getDepth(p(i,:)',R,T);
        A(i,:,:) = [-1/z, 0, p(i,1)/z, p(i,1)*p(i,2), -(1 + p(i,1)^2), p(i,2);
                        0, -1/z, p(i,2)/z, 1+p(i,2)^2, -p(i,1)*p(i,2), -p(i,1)]; 
    end
end

function z = getDepth(p,R,T)
   A = R(1:2,1:2) - sum(R(1:2,3)*p')';
   b = -T(1:2) + p*T(3);
   z = R(1:2,3)'*(A\b) + T(3);
end

function [v] = estimate_velocity(dp,A)
%ESTIMATE_VELOCITY Summary of this function goes here
%   Detailed explanation goes here
    dp = reshape(dp', [numel(dp),1]);
    v = A\dp;
end

function d = error(dp,v,A)
    dp = reshape(dp', [numel(dp),1]);
    e = (A*v - dp);
    d = [e(1:2:end), e(2:2:end)];
    d = vecnorm(d,2,2).^2;
end