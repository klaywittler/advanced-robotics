function [path, num_expanded] = dijkstra(map, start, goal, astar)
% DIJKSTRA Find the shortest path from start to goal.
%   PATH = DIJKSTRA(map, start, goal) returns an mx3 matrix, where each row
%   consists of the (x, y, z) coordinates of a point on the path. The first
%   row is start and the last row is goal. If no path is found, PATH is a
%   0x3 matrix. Consecutive points in PATH should not be farther apart than
%   neighboring voxels in the map (e.g. if 5 consecutive points in PATH are
%   co-linear, don't simplify PATH by removing the 3 intermediate points).
%
%   PATH = DIJKSTRA(map, start, goal, astar) finds the path using euclidean
%   distance to goal as a heuristic if astar is true.
%
%   [PATH, NUM_EXPANDED] = DIJKSTRA(...) returns the path as well as
%   the number of nodes that were expanded while performing the search.
%   
% paramaters:
%   map     - the map object to plan in
%   start   - 1x3 vector of the starting coordinates [x,y,z]
%   goal:   - 1x3 vector of the goal coordinates [x,y,z]
%   astar   - boolean use astar or dijkstra

if nargin < 4
    astar = false;
end
addpath('utils')

%%% parameterize map %%%
res = map.res_xyz;
[xSize,ySize,zSize] = size(map.occgrid);
[j,i,k] = ind2sub(size(map.occgrid), find(map.occgrid <= 1));
[jObs,iObs,kObs] = ind2sub(size(map.occgrid), find(map.occgrid == 1));
Qobs = [jObs, iObs,kObs]; % obstacle vertices from map discretization
Q = [j,i,k]; % vertices from map discretization 
Q(getIndex(Qobs,Q),:) = inf; % set obstacle to inf
parent = nan(numel(Q(:,1)),1); % parents of each node
g = inf(numel(Q(:,1)),1); % array of cost to come
vs = getIndex(pos2sub(map,start),Q); % index of starting position
vg = getIndex(pos2sub(map,goal),Q); % index of goal position

if astar
    h = vecnorm(Q-Q(vg,:),2,2); % euclidean heuristic
%     h = sum((sub2pos(map,Q) - goal).^2,2);
else
    h = zeros(size(g)); % heuristic of zero for dijkstra
end
 
g(vs) = 0; % cost of starting position
f = g + h; % total cost
fMin = 0; % initial cost of 0
u = vs; % initial point to starting position
front = false(size(f)); % set of points to explore (on the frontier)
front(u) = 1; 
explored = false(size(f)); % points already explored
num_expanded = 0; % number of nodes visited
while explored(vg) == 0 && fMin ~= inf % check if goal has been explored and if there are still other points to explore in frontier
    % move point from frontier to explored
    front(u) = 0; 
    explored(u) = 1;
    
    % find neighboring vertices index
    [nIdx, d] = getNeighbors(Q,u,xSize,ySize,zSize,res);
    d(explored(nIdx)) = []; % clear out explored points
    nIdx(explored(nIdx)) = []; 
    front(nIdx) =  1; % add neighbors to frontier
    
    % update cost
    d = g(u) + d; % calculate cost of travel
    lowIdx = d < g(nIdx); % find indices that new path is cheaper
    g(nIdx(lowIdx)) = d(lowIdx); % update cost
    parent(nIdx(lowIdx)) = u; % update parent
    f(nIdx(lowIdx)) = g(nIdx(lowIdx)) + h(nIdx(lowIdx)); % recalculate total cost
    
    % find new point to explore
    uOld = u; % track old index for end of loop
    f(u) = inf; % set explored cost to inf
    [fMin,u] = min(f); % find new point in frontier
    % iterate
    num_expanded = num_expanded + 1;
end

% if no path to goal position available
if ~explored(vg) 
    path = zeros(0,3);
    return
end

% back track path using parents from goal position
u = uOld;
pathIdx = cell(1);
while ~isnan(parent(u))
    pathIdx{1} = [u;pathIdx{1}];
    u = parent(u);
end

path = sub2pos(map,Q(pathIdx{1},:)); % convert path indices to coordinates
path = [start;path;goal]; % append start and goal to fix discretization snap
end

%%%%%%%%%%%%%%%%%%%%%%%
%%% helper function %%%
%%%%%%%%%%%%%%%%%%%%%%%
function ind = getIndex(v,Q)   
% getIndex returns index of element in matrix
% parameters:
%   v - desired element
%   Q - matrix to search
    [~,ind] = ismember(v,Q,'row');
end