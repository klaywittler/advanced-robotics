% data set 1, 4, and 9
% add additional inputs after sensor if you want to
% estimate_pose_handle = @(sensor) estimate_pose(sensor, your_input);
close all; clear all;
addpath('submission')
load('data/studentdata1.mat');
load('aprilTagMap.mat');
warning('off')
K = [311.0520, 0, 201.8724; 0, 311.3885, 113.6210; 0, 0, 1];
Kinv = [311.0520, 0, 201.8724; 0, 311.3885, 113.6210; 0, 0, 1]\eye(3);
T = [-0.04; 0; -0.03];
R = eulzxy2rot([pi,0,-44.5*pi/180]);
  
estimate_pose_handle = @(sensor) estimate_pose(sensor, K, pA, R, T);
estimate_vel_handle = @(sensor) estimate_vel(sensor, Kinv, pA, R, T);

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

figure()
%%% velocity plots
subplot(4,2, 1)
plot(t,vel(1,:),time,vicon(7,:))
xlabel('t')
ylabel('vx [m/s]')
title('linear velocity')
subplot(4,2,3)
plot(t,vel(2,:),time,vicon(8,:))
xlabel('t')
ylabel('vy [m/s]')
subplot(4,2,5)
plot(t,vel(3,:),time,vicon(9,:))
xlabel('t')
ylabel('vz [m/s]')
%%% legend
hSub = subplot(4,2,7.5); plot(1,1,1,1);
hLegend = legend('estimated','vicon');
set(hLegend, 'position', get(hSub, 'position')); % Adjusting legend's position     
axis(hSub,'off');           % Turning its axis off
%%% angular velocity plots
subplot(4,2, 2)
plot(t,omg(1,:),time,vicon(10,:))
xlabel('t')
ylabel('wx [rad/s]')
title('angular velcity')
subplot(4,2,4)
plot(t,omg(2,:),time,vicon(11,:))
xlabel('t')
ylabel('wy [rad/s]')
subplot(4,2, 6)
plot(t,omg(3,:),time,vicon(12,:))
xlabel('t')
ylabel('wz [rad/s]')

[rms_err, ~] = calc_err(time', vicon(7:9,:)', t', vel')

function [rms_err, rms_err_ind] = calc_err(ground_time, ground_data, est_time, est_data)
    int_ground_data = interp1(ground_time, ground_data, est_time);
    rms_err_ind = sqrt(sum((int_ground_data-est_data).^2,1)./numel(est_time));
    rms_err = sqrt(sum((vecnorm(int_ground_data,2,2)-vecnorm(est_data,2,2)).^2,1)./numel(est_time));
end
% function [rms_err, rms_err_ind] = calc_err(ground_time, ground_data, est_time, est_data)
% %     int_ground_data = interp1(ground_time, ground_data, est_time);
% %     rms_err_ind = sqrt(sum((int_ground_data-est_data).^2,1)./numel(est_time));
% %     rms_err = sqrt(sum((vecnorm(int_ground_data,2,2)-vecnorm(est_data,2,2)).^2,1)./numel(est_time));
%     j = 1;
%     rms_err_ind = 0;
%     err = zeros(numel(est_time),1);
%      for i=1:numel(ground_data(1,:))
%         if ground_time(i) == est_time(j)
%             err(j) = norm(ground_data(i)-est_data(j));
%             if j < numel(est_data)
%                 j = j+1;
%             end
%         end
%      end   
%     rms_err = sqrt(sum(err.^2))/numel(est_time);
% end