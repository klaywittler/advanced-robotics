function [v, bestInliers] = motionRANSAC(p, dp, K, R, T)
%RANSAC Summary of this function goes here
%   Detailed explanation goes here
sampleSize = 3;
eps = 10^(-4);
iter = 500;
bestNInliers = 0;
A = getA(p,K,R,T);

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
    end
end

Abest = reshape(permute(A(bestInliers,:,:),[2,1,3]),[bestNInliers*numel(p(1,:)),6]);
v = estimate_velocity(dp(bestInliers,:),p(bestInliers,:),Abest);
disp(['Best number of inliers: ', num2str(bestNInliers), '/', num2str(size(p,1))]); 

end

function d = d(dp,p,v,A)
    dp = reshape(dp', [numel(dp),1]);
    e = (A*v - dp);
    d = [e(1:2:end), e(2:2:end)];
    d = vecnorm(d,2,2).^2;
end

function A = getA(p,K,R,T)
%     b = K(1:2,1:2)*T(1:2);
%     B = [K(1,1);K(2,2)].*R(1:2,1:2);
    
    A = zeros(numel(p(:,1)),2,6);
    for i=1:numel(p(:,1))
%            uc = K(1:2,3) - p(i,:)';
%            Bp = B + sum(R(1:2,3)*uc')';
%            bp = b + uc*T(3);
%            z = R(1:2,3)'*(Bp\bp) + T(3);
           uc = p(i,:)';
           Bp = R(1:2,1:2) + sum(R(1:2,3)*uc')';
           bp = T(1:2) + uc*T(3);
           z = R(1:2,3)'*(Bp\bp) + T(3);
           
           A(i,:,:) = [-1/z, 0, p(i,1)/z, p(i,1)*p(i,2), -(1 + p(i,1)^2), p(i,2);
                            0, -1/z, p(i,2)/z, 1+p(i,2)^2, -p(i,1)*p(i,2), -p(i,1)]; 
    end
end

% function [v] = estimate_velocity(dp,p,A)
% %ESTIMATE_VELOCITY Summary of this function goes here
% %   Detailed explanation goes here
% dp = reshape(dp', [numel(dp),1]);
% v = A\dp;
% end
