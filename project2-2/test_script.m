% data set 1, 4, and 9
% add additional inputs after sensor if you want to
% estimate_pose_handle = @(sensor) estimate_pose(sensor, your_input);
close all; clear all;
addpath('submission')
load('data/studentdata1.mat');
load('aprilTagMap.mat');
K = [311.0520, 0, 201.8724; 0, 311.3885, 113.6210; 0, 0, 1];
T = [-0.04; 0; -0.03];
R = eulzxy2rot([pi,0,-44.5*pi/180]);
  
estimate_pose_handle = @(sensor) estimate_pose(sensor, K, pA, R, T);
estimate_vel_handle = @(sensor) estimate_vel(sensor, K, pA);

pos = zeros(3,numel(data));
q = zeros(4, numel(data));
vel = zeros(3,numel(data));
omg = zeros(3, numel(data));
elapsedTime = zeros(1, numel(data));
profile on
for i=1:numel(data)
    tic
%     [pos(:,i), q(:,i) ] = estimate_pose_handle(data(i)); 
    [vel(:,i), omg(:,i) ] = estimate_vel_handle(data(i)); 
    elapsedTime(i) = toc;
end 
profile report
profile off

disp(['Average run time (ms): ',  num2str(1000*mean(elapsedTime))]);
t = [data.t];
% qVicon = zeros(4,numel(time));
% for i=1:numel(time)
%     qVicon(:,i) = eulzxy2quat([vicon(4,i),vicon(5,i),vicon(6,i)]);
% end

% figure()
% %%% position plots
% subplot(4,2, 1)
% plot(t,pos(1,:),time,vicon(1,:))
% xlabel('t')
% ylabel('x [m]')
% title('position')
% subplot(4,2,3)
% plot(t,pos(2,:),time,vicon(2,:))
% xlabel('t')
% ylabel('y [m]')
% subplot(4,2,5)
% plot(t,pos(3,:),time,vicon(3,:))
% xlabel('t')
% ylabel('z [m]')
% %%% legend
% hSub = subplot(4,2,7); plot(1,1,1,1);
% hLegend = legend('estimated','vicon');
% set(hLegend, 'position', get(hSub, 'position')); % Adjusting legend's position     
% axis(hSub,'off');           % Turning its axis off
% %%% orientation plots
% subplot(4,2, 2)
% plot(t,q(1,:),time,qVicon(1,:))
% xlabel('t')
% ylabel('qw')
% title('orientation')
% subplot(4,2,4)
% plot(t,q(2,:),time,qVicon(2,:))
% xlabel('t')
% ylabel('qx')
% subplot(4,2, 6)
% plot(t,q(3,:),time,qVicon(3,:))
% xlabel('t')
% ylabel('qy')
% subplot(4,2,8)
% plot(t,q(4,:),time,qVicon(4,:))
% xlabel('t')
% ylabel('qz')


figure()
%%% velocity plots
subplot(4,2, 1)
plot(t,vel(1,:),time,vicon(7,:))
xlabel('t')
ylabel('x [m]')
title('position')
subplot(4,2,3)
plot(t,vel(2,:),time,vicon(8,:))
xlabel('t')
ylabel('y [m]')
subplot(4,2,5)
plot(t,vel(3,:),time,vicon(9,:))
xlabel('t')
ylabel('z [m]')
%%% legend
hSub = subplot(4,2,7.5); plot(1,1,1,1);
hLegend = legend('estimated','vicon');
set(hLegend, 'position', get(hSub, 'position')); % Adjusting legend's position     
axis(hSub,'off');           % Turning its axis off
%%% angular velocity plots
subplot(4,2, 2)
plot(t,omg(1,:),time,vicon(10,:))
xlabel('t')
ylabel('wx')
title('orientation')
subplot(4,2,4)
plot(t,omg(2,:),time,vicon(11,:))
xlabel('t')
ylabel('wy')
subplot(4,2, 6)
plot(t,omg(3,:),time,vicon(12,:))
xlabel('t')
ylabel('wz')