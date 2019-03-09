% Add student code to path
addpath(genpath('student_code'))
% Add path to framework
addpath(genpath('framework'))

% Now you can start your quadrotor like this:
%
% Create quad object to start quadrotor control
% quad = QuadrotorROS('meam04');
%
% Then take off:
%
% quad.takeoff

%% Maze navigation functions

% Load the map
map = load_map('maps/maze.txt', 0.1, 0.1, 0.2);

% Generate goal points
[goals, starts] = make_maze_goals;

% To plan and plot a path from the current location to a goal point:
% quad.show_path(map, goals{1})

% To plan a path, then generate a trajectory and plot the desired 
% positions, velocities, and accelerations without flying
% quad.drynavigate(map, goals{1})

% To look at the trajectory planned and followed overlaid on 
% the map:
% quad.plot_maze(map)

% To navigate the maze to the first goal point:
% quad.navigate(map, goals{1})


