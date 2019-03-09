% Add student code to path
addpath(genpath('student_code'))
% Add path to framework
addpath(genpath('framework'))
% Load the map
map = load_map('maps/maze.txt', 0.1, 0.1, 0.2);

astar = true;
% Generate goal points
[goals, starts] = make_maze_goals;
wpt = [];
% wpt = dijkstra(map, starts{2}, goals{1}, astar);

% Load data
load('data/dijkstra_goal1.mat')
% Plotting
% plot_run(h)
% plot_3d(h, wpt)
% plot_saved_maze_data(h, map)
