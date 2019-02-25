close all;
clear all;
clc;
addpath(genpath('./'));

%% Plan path
disp('Planning ...');
map = load_map('mymap.txt', 0.1, 1, 0.25);
% start = {[0, 6, 2]};
% stop = {[20, 0, 2]};
% start = {[-2.5 0.0 0.0]};
% stop = {[3 -1.0 1.0]};
start = {[0.0, 0, 0.2]};
% stop = {[10.0, 10.0, 2.0]};
stop = {[4.0, 18.0, 1.0]};
nquad = length(start);
for qn = 1:nquad
    path{qn} = dijkstra(map, start{qn}, stop{qn}, true);
end
if nquad == 1
    plot_path(map, path{1});
else
    % you could modify your plot_path to handle cell input for multiple robots
end

%% Generate trajectory
disp('Generating Trajectory ...');
profile on
trajectory_generator([], [], map, path{1});
profile report
profile off

%% Run trajectory
disp('Running Trajectory ...');
trajectory = test_trajectory(start, stop, map, path, true); % with visualization
