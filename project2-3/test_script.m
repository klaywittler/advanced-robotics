% Add additional inputs after the given ones if you want to
% Example:
% your_input = 1;
% ekf_handle1 = @(sensor, vic) ekf1(sensor, vic, your_input);
% ekf_handle2 = @(sensor) ekf2(sensor, your_input);
%
% We will only call ekf_handle in the test function.
% Note that this will only create a function handle, but not run the function
close all; clear all;
addpath('submission')
load('data/studentdata1.mat');
load('aprilTagMap.mat');

Kinv = [311.0520, 0, 201.8724; 0, 311.3885, 113.6210; 0, 0, 1]\eye(3);
T = [-0.04; 0; -0.03];
R = eulzxy2rot([pi,0,-44.5*pi/180]);

estimate_pose_handle = @(sensor) estimate_pose(sensor,Kinv, pA, R, T);
estimate_vel_handle = @(sensor) estimate_vel(sensor, Kinv, pA, R, T);
ekf1_handle = @(sensor, vic) ekf1(sensor, vic, Kinv, pA, R, T);
ekf2_handle = @(sensor) ekf2(sensor, Kinv, pA, R, T);

X1 = zeros(7,numel(vicon(1,:)));
Z1 = zeros(9,numel(vicon(1,:)));
X2 = zeros(15,numel(vicon(1,:)));
Z2 = zeros(15,numel(vicon(1,:)));
j = 1;
elapsedTime = zeros(1, numel(data));
profile on
for i=1:numel(vicon(1,:))-1
    if time(i) == data(j).t
        d = data(j);
        j = j+1;
    else
        d.id = [];
    end
    v.vel = vicon(7:12,i);
    v.t = time(i);
    tic
%     [pos(:,i), q(:,i) ] = estimate_pose_handle(data(i)); 
%     [vel(:,i), omg(:,i) ] = estimate_vel_handle(data(i)); 
%     [X1(:,i), Z1(:,i)] = ekf1_handle(d, v);
    [X2(:,i), Z2(:,i)] = ekf2_handle(d);
    elapsedTime(i) = toc;
end 
profile report
profile off

disp(['Average run time (ms): ',  num2str(1000*mean(elapsedTime))]);
t = [data.t];
qVicon = zeros(4,numel(time));
for i=1:numel(time)
    qVicon(:,i) = eulzxy2quat([vicon(4,i),vicon(5,i),vicon(6,i)]);
end

figure()
%%% x1 plots
subplot(4,2, 1)
plot(time,X1(1,:),time,vicon(1,:))
xlabel('t')
ylabel('x [m]')
title('position')
subplot(4,2,3)
plot(time,X1(2,:),time,vicon(2,:))
xlabel('t')
ylabel('y [m]')
subplot(4,2,5)
plot(time,X1(3,:),time,vicon(3,:))
xlabel('t')
ylabel('z [m]')
%%% legend
hSub = subplot(4,2,7.5); plot(1,1,1,1);
hLegend = legend('estimated','vicon');
set(hLegend, 'position', get(hSub, 'position')); % Adjusting legend's position     
axis(hSub,'off');           % Turning its axis off
%%% angular velocity plots
subplot(4,2, 2)
plot(time,X1(4,:),time,qVicon(1,:))
xlabel('t')
ylabel('w')
title('orientation')
subplot(4,2,4)
plot(time,X1(5,:),time,qVicon(2,:))
xlabel('t')
ylabel('qx')
subplot(4,2, 6)
plot(time,X1(6,:),time,qVicon(3,:))
xlabel('t')
ylabel('qy')
subplot(4,2, 8)
plot(time,X1(7,:),time,qVicon(4,:))
xlabel('t')
ylabel('qz')

figure()
%%% x2 plots
subplot(4,2, 1)
plot(time,X2(1,:),time,vicon(1,:))
xlabel('t')
ylabel('x [m]')
title('position')
subplot(4,2,3)
plot(time,X2(2,:),time,vicon(2,:))
xlabel('t')
ylabel('y [m]')
subplot(4,2,5)
plot(time,X2(3,:),time,vicon(3,:))
xlabel('t')
ylabel('z [m]')
%%% legend
hSub = subplot(4,2,7.5); plot(1,1,1,1);
hLegend = legend('estimated','vicon');
set(hLegend, 'position', get(hSub, 'position')); % Adjusting legend's position     
axis(hSub,'off');           % Turning its axis off
%%% angular velocity plots
subplot(4,2, 2)
plot(time,X2(4,:),time,qVicon(1,:))
xlabel('t')
ylabel('w')
title('orientation')
subplot(4,2,4)
plot(time,X2(5,:),time,qVicon(2,:))
xlabel('t')
ylabel('qx')
subplot(4,2, 6)
plot(time,X2(6,:),time,qVicon(3,:))
xlabel('t')
ylabel('qy')
subplot(4,2, 8)
plot(time,X2(7,:),time,qVicon(4,:))
xlabel('t')
ylabel('qz')


%%& error
% [rms_err, rms_err_ind] = calc_err(time', vicon(7:9,:)', t', vel');
% 
% function [rms_err, rms_err_ind] = calc_err(ground_time, ground_data, est_time, est_data)
%     int_ground_data = interp1(ground_time, ground_data, est_time);
%     rms_err_ind = sqrt(sum((int_ground_data-est_data).^2,1)./numel(est_time));
%     rms_err = sqrt(sum((vecnorm(int_ground_data,2,2)-vecnorm(est_data,2,2)).^2,1)./numel(est_time));
% end