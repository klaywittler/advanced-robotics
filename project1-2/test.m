close all
clear all

addpath('sample_maps')
addpath('utils')

astar = true;
map = load_map('map0.txt',0.5,1,0.01);
[path, num_expanded] = dijkstra(map,[0,3,5],[20,3,5],astar);
%[path, num_expanded] = dijkstra(map,[0;-5;5],[5;20;3]);
plot_path(map,path);