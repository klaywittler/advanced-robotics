function [v, bestInliers] = motionRANSAC(p, dp, H)
% Estimate velocity given a set of 
% pairs of matching *calibrated* points
% X1,X2: Nx2 matrices of calibrated points
%   i^th row of X1 matches i^th row of X2
%
% E: robustly estimated E matrix
% bestInliers: indices of the rows of X1 (and X2) that where in the
% largest consensus set
sampleSize = 3;
eps = 5*10^(-4);
iter = 100;
bestNInliers = 0;
minNInliers = size(p,1)*0.5;
bestError = 10000;
A = getA(p,H);

for i=1:iter
    indices = randperm(size(p,1));
    sampleInd = indices(1:sampleSize);
    testInd =  indices(sampleSize+1:length(indices));
    Asample = reshape(permute(A(sampleInd,:,:),[2,1,3]),[sampleSize*numel(p(1,:)),6]);
    Atest = reshape(permute(A(testInd,:,:),[2,1,3]),[(numel(p(:,1))-sampleSize)*numel(p(1,:)),6]);
    
    v_sample = estimate_velocity(dp(sampleInd,:),Asample);
    
    [residuals, avgError] = error(dp(testInd,:),v_sample,Atest); 
    
    curInliers = [testInd(residuals < eps), sampleInd];            % don't forget to include the sampleInd
    
    curNInliers = length(curInliers);

    if curNInliers > bestNInliers || (avgError < bestError && curNInliers >= minNInliers)
        bestNInliers = curNInliers;
        bestInliers = curInliers; 
        bestError = avgError;
    end
end

Abest = reshape(permute(A(bestInliers,:,:),[2,1,3]),[bestNInliers*numel(p(1,:)),6]);
v = estimate_velocity(dp(bestInliers,:),Abest);
% disp(['Best number of inliers: ', num2str(bestNInliers), '/', num2str(size(p,1))]); 

end

function A = getA(p,H)
    A = zeros(numel(p(:,1)),2,6);
    z = getDepth(p',H);
    zr0 = zeros(size(z));
    A(:,1,:) = [-1./z, zr0, p(:,1)./z, p(:,1).*p(:,2), -(1 + p(:,1).^2), p(:,2)];
    A(:,2,:) = [zr0, -1./z, p(:,2)./z, 1+p(:,2).^2, -p(:,1).*p(:,2), -p(:,1)];
end

function z = getDepth(p,H)
    x = H\[p;ones(1,numel(p(1,:)))];
    x = x./x(3,:);
    z = (H(3,:)*x)';
end

function [v] = estimate_velocity(dp,A)
%ESTIMATE_VELOCITY Summary of this function goes here
%   Detailed explanation goes here
    dp = reshape(dp', [numel(dp),1]);
    v = A\dp;
end

function [error, avg] = error(dp,v,A)
    dp = reshape(dp', [numel(dp),1]);
    e = (A*v - dp);
    d = reshape(e, [2,numel(dp)/2])';
    error = sum(d.^2,2);
    avg = sum(error)/numel(error);
end