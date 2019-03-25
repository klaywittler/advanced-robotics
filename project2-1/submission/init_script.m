% add additional inputs after sensor if you want to
% Example:
% your_input = 1;
% estimate_pose_handle = @(sensor) estimate_pose(sensor, your_input);
% We will only call estimate_pose_handle in the test function.
% Note that unlike project 1 phase 3, thise will only create a function
% handle, but not run the function at all.

% load('data/studentdata1.mat');
load('aprilTagMap.mat');
K = [311.0520, 0, 201.8724; 0, 311.3885, 113.6210; 0, 0, 1];
T = [-0.04; 0; -0.03];
R = eulzxy2rot([0,pi,-pi/4]);
  
estimate_pose_handle = @(sensor) estimate_pose(sensor,K, pA, R, T);