% add additional inputs after sensor if you want to
% Example:
% your_input = 1;
% estimate_pose_handle = @(sensor) estimate_pose(sensor, your_input);
% We will only call estimate_pose_handle in the test function.
% Note that unlike project 1 phase 3, thise will only create a function
% handle, but not run the function at all.
close all; clear all;
addpath('submission')
load('data/studentdata1.mat');
load('aprilTagMap.mat');
K = [311.0520, 0, 201.8724; 0, 311.3885, 113.6210; 0, 0, 1];
T = [-0.04; 0; -0.03];
R = eulzxy2rot([pi,0,-44.5*pi/180]);
  
estimate_pose_handle = @(sensor) estimate_pose(sensor,K, pA, R, T);

pos = zeros(3,numel(data));
q = zeros(4, numel(data));
for i=1:numel(data)
    [pos(:,i), q(:,i) ] = estimate_pose_handle(data(i));   
end
 
t = [data.t];
% figure()
% subplot(3,1, 1)
% plot(t,pos(1,:),time,vicon(1,:))
% ylabel('x')
% subplot(3,1,2)
% plot(t,pos(2,:),time,vicon(2,:))
% ylabel('y')
% subplot(3,1,3)
% plot(t,pos(3,:),time,vicon(3,:))
% ylabel('z')
% legend('estimated','vicon')

qVicon = zeros(4,numel(time));
for i=1:numel(time)
    qVicon(:,i) = eulzxy2quat([vicon(4,i),vicon(5,i),vicon(6,i)]);
end

figure()
subplot(4,1, 1)
plot(t,q(1,:),time,qVicon(1,:))
ylabel('qw')
subplot(4,1,2)
plot(t,q(2,:),time,qVicon(2,:))
ylabel('qx')
subplot(4,1, 3)
plot(t,q(3,:),time,qVicon(3,:))
ylabel('qy')
subplot(4,1,4)
plot(t,q(4,:),time,qVicon(4,:))
ylabel('qz')