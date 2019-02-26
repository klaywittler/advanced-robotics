close all;
clear all;
clc;
addpath(genpath('./'));

%% Plan path
disp('Planning ...');
map = load_map('mymap.txt', 0.1, 1, 0.25);
% map = load_map('map1.txt', 0.1, 1, 0.25);
% start = {[0, 6, 2]}; % map4
% stop = {[20, 0, 2]}; % map4
% start = {[-2.5 0.0 0.0]};
% stop = {[3 -1.0 1.0]};
start = {[0.0, 0, 0.2]}; % my map
stop = {[12.0, 2.0, 5.5]}; % my map

% start = {[0.0, -4.9, 0.2]}; % map 1
% stop = {[8.0, 18.0, 3.0]}; % map 1

nquad = length(start);
for qn = 1:nquad
    profile on
    path{qn} = dijkstra(map, start{qn}, stop{qn}, true);
end
if nquad == 1
    plot_path(map, path{1});
else
    % you could modify your plot_path to handle cell input for multiple robots
end

%% Generate trajectory
disp('Generating Trajectory ...');
trajectory_generator([], [], map, path{1});
profile report
profile off

%% Run trajectory
disp('Running Trajectory ...');
trajectory = test_trajectory(start, stop, map, path, true); % with visualization
