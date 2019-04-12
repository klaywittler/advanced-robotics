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

ekf1_handle = @(sensor, vic) ekf1(sensor, vic);
ekf2_handle = @(sensor) ekf2(sensor);


elapsedTime = zeros(1, numel(data));
profile on
for i=1:numel(data)
    tic
%     [pos(:,i), q(:,i) ] = estimate_pose_handle(data(i)); 
%     [vel(:,i), omg(:,i) ] = estimate_vel_handle(data(i)); 
    elapsedTime(i) = toc;
end 
profile report
profile off

disp(['Average run time (ms): ',  num2str(1000*mean(elapsedTime))]);
t = [data.t];








%%& error
% [rms_err, rms_err_ind] = calc_err(time', vicon(7:9,:)', t', vel');
% 
% function [rms_err, rms_err_ind] = calc_err(ground_time, ground_data, est_time, est_data)
%     int_ground_data = interp1(ground_time, ground_data, est_time);
%     rms_err_ind = sqrt(sum((int_ground_data-est_data).^2,1)./numel(est_time));
%     rms_err = sqrt(sum((vecnorm(int_ground_data,2,2)-vecnorm(est_data,2,2)).^2,1)./numel(est_time));
% end