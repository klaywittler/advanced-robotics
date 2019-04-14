function v = motionRANSAC(p, dp, H, R, T)
% Estimate velocity given a set of 
% pairs of matching *calibrated* points
% X1,X2: Nx2 matrices of calibrated points
%   i^th row of X1 matches i^th row of X2
%
% E: robustly estimated E matrix
% bestInliers: indices of the rows of X1 (and X2) that where in the
% largest consensus set
sampleSize = 3;
eps = 5*10^(-3);
iter = 90;
bestNInliers = 0;
AB = getAB(p,H, R, T);

for i=1:iter
    indices = randperm(size(p,1));
    sampleInd = indices(1:sampleSize);
    testInd =  indices(sampleSize+1:length(indices));
    ABsample = reshape(permute(AB(sampleInd,:,:),[2,1,3]),[sampleSize*numel(p(1,:)),6]);
    ABtest = reshape(permute(AB(testInd,:,:),[2,1,3]),[(numel(p(:,1))-sampleSize)*numel(p(1,:)),6]);
    
    v_sample = estimate_velocity(dp(sampleInd,:),ABsample);
    
    residuals = error(dp(testInd,:),v_sample,ABtest); 
    
    curInliers = [testInd(residuals < eps), sampleInd];
    
    curNInliers = length(curInliers);
    
    if curNInliers > bestNInliers
        bestNInliers = curNInliers;
        bestInliers = curInliers; 
        if bestNInliers == size(p,1)
            break; 
        end
    end
end

ABbest = reshape(permute(AB(bestInliers,:,:),[2,1,3]),[bestNInliers*numel(p(1,:)),6]);
v = estimate_velocity(dp(bestInliers,:),ABbest);
% disp(['Best number of inliers: ', num2str(bestNInliers), '/', num2str(size(p,1))]); 

end

function AB = getAB(p, H, R, T)
    AB = zeros(numel(p(:,1)),2,6);
    z = getDepth(p',H, R, T);
    zr0 = zeros(size(z));
    AB(:,1,:) = [-1./z, zr0, p(:,1)./z, p(:,1).*p(:,2), -(1 + p(:,1).^2), p(:,2)];
    AB(:,2,:) = [zr0, -1./z, p(:,2)./z, 1+p(:,2).^2, -p(:,1).*p(:,2), -p(:,1)];
end

function z = getDepth(p,H,R,T)
    x = H\[p;ones(1,numel(p(1,:)))];
    x = x./x(3,:);
    xc = [R(1:3,1:2), T]*x;
    z = xc(3,:)';
end

function [v] = estimate_velocity(dp,AB)
    dp = reshape(dp', [numel(dp),1]);
    v = AB\dp;
end

function error = error(dp,v,AB)
    dp = reshape(dp', [numel(dp),1]);
    e = (AB*v - dp);
    d = reshape(e, [2,numel(dp)/2])';
    error = sum(d.^2,2);
end