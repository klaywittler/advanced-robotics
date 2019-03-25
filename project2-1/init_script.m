% add additional inputs after sensor if you want to
% Example:
% your_input = 1;
% estimate_pose_handle = @(sensor) estimate_pose(sensor, your_input);
% We will only call estimate_pose_handle in the test function.
% Note that unlike project 1 phase 3, thise will only create a function
% handle, but not run the function at all.

load('data/studentdata1.mat');
K = [311.0520, 0, 201.8724; 0, 311.3885, 113.6210; 0, 0, 1];
T = [-0.04; 0; -0.03];
R = eulzxy2rot([0,pi,pi/4]);

p = zeros(2,5,12*9);
pInit0 = [0.152/2, 0.152, 0.152, 0, 0; 0.152/2, 0, 0.152, 0.152, 0];
c = 1;
for j=1:9
    pInit = pInit0;
    for i=1:12
        p(:,:,c) = pInit;
        pInit = pInit + 0.152*[ones(1,5);zeros(1,5)];
        c = c + 1;
    end 
    if j== 3 || j == 6
        pInit0 = pInit0 + 0.178*[zeros(1,5);ones(1,5)]; 
    else
        pInit0 = pInit0 + 0.152*[zeros(1,5);ones(1,5)];
    end
end
  
estimate_pose_handle = @(sensor) estimate_pose(sensor,K, p, R, T);

pos = zeros(3,numel(data));
q = zeros(4, numel(data));
profile on
for i=1:numel(data)
    [pos(:,i), q(:,i) ] = estimate_pose_handle(data(i));   
end
profile report
profile off

t = [data.t];
figure()
subplot(3,1, 1)
plot(t,pos(1,:),time,vicon(1,:))
ylabel('x')
subplot(3,1,2)
plot(t,pos(2,:),time,vicon(2,:))
ylabel('y')
subplot(3,1,3)
plot(t,pos(3,:),time,vicon(3,:))
ylabel('z')

% qVicon = zeros(4,numel(time));
% for i=1:numel(time)
%     qVicon(:,i) = eulzxy2quat([vicon(4,i),vicon(5,i),vicon(6,i)]);
% end
% 
% figure()
% subplot(4,1, 1)
% plot(t,q(1,:),time,qVicon(1,:))
% subplot(4,1,2)
% plot(t,q(2,:),time,qVicon(2,:))
% subplot(4,1, 1)
% plot(t,q(3,:),time,qVicon(3,:))
% subplot(4,1,2)
% plot(t,q(4,:),time,qVicon(4,:))