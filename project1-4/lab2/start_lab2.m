% Add student code to path
addpath(genpath('student_code'))
% Add path to framework
addpath(genpath('framework'))

% Load the map
map = load_map('maps/maze.txt', 0.1, 0.1, 0.2);

% Generate goal points
[goals, starts] = make_maze_goals;

%% Step 1 Check controller stability
% Open an emergency stop button
% Type the name of the quadrotor 'meamXX'
% Click connect, battery voltage should appear
% Right click on top bar, select always on top

% Create quad object
% quad = QuadrotorROS('meam04');
%
% Then take off:
% quad.takeoff

% Switch to your controller
% quad.shover

% Land:
% quad.land; 

%% Checking Dijkstra and Trajectory Generator

% To see the dijkstra plan to the goal
% quad.show_path(map, goals{1})

% Create quad object to clear the saved data
% quad = QuadrotorROS('meam04');

% To start publishing desired trajectory states without flying
% quad.drynavigate(map, goals{1})

% Look at plots of desired states. Ensure continuity and v < 0.5
% quad.plot

% To look at the planned trajectory overlaid on the map:
% quad.plot_maze(map)

%% Flying to the goal

% Create quad object to clear the saved data
% quad = QuadrotorROS('meam04');

% take off:
% quad.takeoff

% To navigate the maze to the goal:
% quad.navigate(map, goals{1})

%% Saving data

% Save your data using
% quad.save_data('goal1_left')

% Clear your previous data by making a new quad object
% quad = QuadrotorROS('meam04');

% h1 = load('data/goal1_left');


