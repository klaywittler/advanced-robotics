% Add additional inputs after sensor if you want to
% Example:
% your_input = 1;
% estimate_vel_handle = @(sensor) estimate_vel(sensor, your_input);
%
% We will only call estimate_vel_handle in the test function.
% Note that thise will only create a function handle, but not run the function
warning('off');
load('aprilTagMap.mat');
Kinv = [311.0520, 0, 201.8724; 0, 311.3885, 113.6210; 0, 0, 1]\eye(3);
T = [-0.04; 0; -0.03];
R = eulzxy2rot([pi,0,-44.5*pi/180]);

estimate_vel_handle = @(sensor) estimate_vel(sensor, Kinv, pA, R, T);
