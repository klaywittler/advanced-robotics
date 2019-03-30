function [v, bestInliers] = motionRANSAC(p,dp)
%RANSAC Summary of this function goes here
%   Detailed explanation goes here
sampleSize = 3;
eps = 10^(-3);
iter = 500;
bestNInliers = 0;
A = getA(p);

for i=1:iter
    indices = randperm(size(p,1));
    sampleInd = indices(1:sampleSize);
    testInd =  indices(sampleSize+1:length(indices));
    Asample = reshape(permute(A(sampleInd,:,:),[2,1,3]),[sampleSize*numel(p(1,:)),6]);
    Atest = reshape(permute(A(testInd,:,:),[2,1,3]),[(numel(p(:,1))-sampleSize)*numel(p(1,:)),6]);
    
    v_sample = estimate_velocity(dp(sampleInd,:),p(sampleInd,:),Asample);
    
    residuals = d(dp(testInd,:),p(testInd,:),v_sample,Atest); 
    
    curInliers = [testInd(residuals < eps), sampleInd];            % don't forget to include the sampleInd
    
    curNInliers = length(curInliers);

    if curNInliers > bestNInliers
        bestNInliers = curNInliers;
        bestInliers = curInliers;
%         v = v_sample;
        v = estimate_velocity(dp(curInliers,:),p(curInliers,:));
    end
end

disp(['Best number of inliers: ', num2str(bestNInliers), '/', num2str(size(X1,1))]); 

end

function d = d(dp,p,v,A)
dp = reshape(dp', [numel(dp),1]);
%     A = zeros(numel(dp),6);
%     for i=1:2:numel(dp)
%     A(i:i+1,:) = [-1*z, 0, p(1)*z, p(1)*p(2), -(1 + p(1)^2), p(2);
%                     0, -1*z, p(2)*z, 1+p(2)^2, -p(1)*p(2), -p(1)];
%     end
    e = (A*v - dp);
    d = [e(1:2:end), e(2:2:end)];
    d = vecnorm(d,2,2).^2;
end

function A = getA(p)
    z = 1;
    A = zeros(numel(p(:,1)),2,6);
    for i=1:numel(p(:,1))
           A(i,:,:) = [-1*z, 0, p(i,1)*z, p(i,1)*p(i,2), -(1 + p(i,1)^2), p(i,2);
                            0, -1*z, p(i,2)*z, 1+p(i,2)^2, -p(i,1)*p(i,2), -p(i,1)]; 
    end
end

