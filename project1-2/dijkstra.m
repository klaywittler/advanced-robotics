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

addpath('utils')

num_expanded = 0;

[j,i,k] = ind2sub(size(map.occgrid), find(map.occgrid == 0));
Q = [j,i,k];
g = inf(numel(Q(:,1)),1);
p = nan(numel(Q(:,1)),1);
vs = getIndex(pos2sub(map,start),Q);
vg = getIndex(pos2sub(map,goal),Q);

g(vs) = 0;

if nargin < 4
    astar = false;
end

if astar
    h = vecnorm(Q-Q(vg,:),2,2);
else
    h = 0;
end
    
f = g + h;

[M,u] = min(f);
front = u;
old = [];
% hold on;
while ~ismember(vg,old) && M ~= inf
    front = setdiff(front,u);
    old = [old;u];
    
    [nIdx, nDist] = knnsearch(Q,Q(u,:),'K',27, 'IncludeTies', true);
    nIdx = nIdx{1}(nDist{1} <= sqrt(2)).';
    nIdx = setdiff(nIdx,old);
    d = g(u) + ones(size(nIdx));
    lowIdx = d < g(nIdx);
    g(nIdx(lowIdx)) = d(lowIdx);
    p(nIdx(lowIdx)) = u;
    
%     scatter3(Q(u,2),Q(u,1),Q(u,3))
%     axis([0 20 0 5 0 6])
%     pause(0.1)
%     disp(u);
    
    front = [front; setdiff(nIdx,front)];
    b = inf(size(f));
    b(front) = 1;
    
    f = g + h;
    uOld = u;
    [M,u] = min(f.*b);
    
    num_expanded = num_expanded + 1;
end

u = uOld;
pathIdx = [];
while ~isnan(p(u))
    pathIdx = [u;pathIdx];
    u = p(u);
end

path = sub2pos(map,Q(pathIdx,:));

end

function ind = getIndex(v,Q)   
    [~,ind] = ismember(v,Q,'row');
end
