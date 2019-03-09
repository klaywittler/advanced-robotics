function [goals, starts] = make_maze_goals
%
% [goals] = make_maze_goals
%
% returns cell array of size 3. Each cell contains
% a sequence of waypoints, which are stored in
% a nx3 matrix. Each row contains one waypoint;

goals{1} = [2.4 1.2 1];
goals{2} = [2.4 -1.4 1];
starts{1} = [-1.75 1 1];
starts{2} = [-1.75 -1 1];
end