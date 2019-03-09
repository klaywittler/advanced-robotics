% Add student code to path
addpath(genpath('student_code'))
% Add path to framework
addpath(genpath('framework'))

% Load the map
map = load_map('maps/maze.txt', 0.1, 0.1, 0.2);


%
% Now you can start your quadrotor like this:
%
% Create quad object to start quadrotor control
% quad = QuadrotorROS('meam04');
%
% Then take off:
%
% quad.takeoff
%


