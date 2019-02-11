close all
clear all

addpath('sample_maps')
addpath('utils')
addpath('submission')

astar = false;
% map = load_map('map0.txt',0.5,1,0.1);
map = load_map('map1.txt',0.1,1,0.1);
profile on
% [path, num_expanded] = dijkstra(map,[0,3,5],[20,3,5],astar);
[path, num_expanded] = dijkstra(map,[0,-5,3],[5,20,5],astar);
profile report
profile off
plot_path(map,path);
 