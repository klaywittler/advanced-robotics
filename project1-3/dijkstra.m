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
%     h = vecnorm(Q-Q(vg,:),1,2); % euclidean heuristic
    h = sum((sub2pos(map,Q) - goal).^2,2); % l2
%     h = sum(abs(sub2pos(map,Q) - goal),2); % l1
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
path = pruneMap(path, map);
path = [start;path;goal]; % append start and goal to fix discretization snap
path = unique(path,'rows','stable');

end

%%%%%%%%%%%%%%%%%%%%%%%%
%%% helper function  %%%
%%%%%%%%%%%%%%%%%%%%%%%%
function ind = getIndex(v,Q)   
% getIndex returns index of element in matrix
% parameters:
%   v - desired element
%   Q - matrix to search
    [~,ind] = ismember(v,Q,'row');
end

%%%%%%%%%%%%%%%%%%%%%%%%
%%% helper functions %%%
%%%%%%%%%%%%%%%%%%%%%%%%
function [neighbors, cost] = getNeighbors(positions, index, xSize, ySize, zSize, res)
% getNeighbors returns indices for neighbors to an index and cost(distance)
% to tavel to point
% parameters:
%   positions - nx3 matrix of vertices represented in [i,j,k] indices for the map
%   index - vertex to find neighbors for
%   xSize,ySize,zSize - size of map in x,y,z direction
%   res - 3x1 vector of map resolution in x,y,z directions

%%% z %%%
if (zSize == 1) % if empty cannot move in direction
    add1 = 0;
    cadd1 = 0;
elseif (positions(index,3) == zSize) % outer boundary - move back or stay still
    add1 = [-xSize*ySize;0]; % z index increaments every xSize*ySize
    cadd1 = [res(3);0].^2;
elseif (positions(index,3) == 1) % inner boundary - stay still or move forware 
    add1 = [0; xSize*ySize]; % z index increaments every xSize*ySize
    cadd1 = [0;res(3)].^2;
else % free to move
    add1= [-xSize*ySize;0;xSize*ySize]; % z index increaments every xSize*ySize
    cadd1 = [res(3);0;res(3)].^2;
end

%%% y %%%
if (ySize == 1) % if empty cannot move in direction
    add2 = 0;
    cadd2 = 0;
elseif (positions(index,2) == ySize) % outer boundary - move back or stay still
    add2 = [-xSize;0]; % y index increaments every xSize
    cadd2 = [res(2);0].^2;
elseif (positions(index,2) == 1) % inner boundary - stay still or move forware
    add2 = [0; xSize]; % y index increaments every xSize
    cadd2 = [0;res(2)].^2;
else % free to move
    add2 = [-xSize;0;xSize]; % y index increaments every xSize
    cadd2 = [res(2);0;res(2)].^2;
end

%%% x %%%
if (xSize == 1) % if empty cannot move in direction
    add3 = 0;
    cadd3 = 0;
elseif (positions(index,1) == xSize) % outer boundary - move back or stay still
    add3 = [-1;0]; % x index increaments everytime
    cadd3 = [res(1);0].^2;
elseif (positions(index,1) == 1) % inner boundary - stay still or move forware
    add3 = [0; 1]; % x index increaments everytime
    cadd3 = [0;res(1)].^2;
else % free to move
    add3 = [-1;0;1]; % x index increaments everytime
    cadd3 = [res(1);0;res(1)].^2;
end

neighbors = (index + elementwise(add3, elementwise(add2,add1))); % convole possible movements
cost = elementwise(cadd3, elementwise(cadd2,cadd1)); 
cost(isinf(positions(neighbors,1))) = []; % get rid of obstacle entries
cost = sqrt(cost);
neighbors(isinf(positions(neighbors,1))) = []; % get rid of obstacle entries
end

function array = elementwise(a1,a2)
% elementwise convoles the two arrays
% parameters:
%   a1 - 1xn vetor
%   a2 - 1xn vector
    array = a1.' + a2;
    array = array(:);
end

%%%%%%%%%%%%%%%%%%%%%%%%
%%% helper functions %%%
%%%%%%%%%%%%%%%%%%%%%%%%
function pruned = pruneMap(path, map)
%     pruned_f = pruneForward(path, map, center, boundary);
    pruned_temp = pruneBackward(path, map);
%     dist_f = sum(sqrt(sum(diff(pruned_f, 1).^2,2)));
%     dist_b = sum(sqrt(sum(diff(pruned_b, 1).^2,2)));
%     if dist_f >= dist_b
%         pruned_temp = pruned_b;
%     else
%         pruned_temp = pruned_f;
%     end 
    pruned = pruned_temp(1,:);
    for i=1:numel(pruned_temp(:,1))-1
        current = pruned_temp(i,:);
        next = pruned_temp(i+1,:);
        d = sqrt(sum((next-current).^2));
        pruned = [pruned;generatePoints(current, next, 7*d*map.res_xyz)];
    end
    pruned = [pruned;pruned_temp(end,:)];
end

%Forwards
function pruned = pruneForward(path, map)
    [center,boundary] = getBoxes(map.blocks);
    path_index = 1;
    current = path(path_index, :);
    pruned = [current];
    while ~all(current == path(end, :))
        next_index = path_index+1;
        next = path(next_index, :);
        while (next_index + 1 <= numel(path(:,1))) && ~checkCollision(map, generatePoints(current, next, map.res_xyz), center, boundary)
            next_index = next_index + 1;
            next = path(next_index, :);
        end
        path_index = next_index ;
        current = path(path_index, :);
        pruned = [pruned; current];
    end
end

%Backwards
function pruned = pruneBackward(path, map)
    [center,boundary] = getBoxes(map.blocks);
    path_index = 1;
    current = path(path_index, :);
    pruned = [current];
    while ~all(current == path(end, :))
        next_index = numel(path(:,1));
        next = path(next_index, :);
        while (next_index - 1 ~= path_index) && checkCollision(map, generatePoints(current, next, map.res_xyz), center, boundary)
            next_index = next_index - 1;
            next = path(next_index, :);
        end
        path_index = next_index;
        current = path(path_index, :);
        pruned = [pruned; current];
    end
end

function collides = checkCollision(map, points, c, b)
    collides = false;
%     [c,b] = getBoxes(map.blocks);
%     p = reshape(points,[1,3,numel(points(:,1))]);
%     if any(all(p <= c + b + map.margin,2) & all(p >= c - b - map.margin,2),'all')
%        collides = true;
%     end
    for ii = 1:numel(points(:, 1))
        if any(all(points(ii,:) <= c + b + map.margin,2) & all(points(ii,:) >= c - b - map.margin,2))
           collides = true;
           return
        end
    end
end

function points = generatePoints(p1, p2, res)
    disc = max(abs(p1 - p2)./(res/10));
    x = linspace(p1(1), p2(1), disc).';
    y = linspace(p1(2), p2(2), disc).';
    z = linspace(p1(3), p2(3), disc).';
    points = [x, y, z];
end

function [c,b] = getBoxes(box)
    c = (box(:,1:3)+box(:,4:6))/2;
    b = c-box(:,1:3);
end