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
res = map.res_xyz;
[xSize,ySize,zSize] = size(map.occgrid);

if xSize == 0 || ySize == 0 || zSize == 0
   path = [start;goal];
   return
end

[j,i,k] = ind2sub(size(map.occgrid), find(map.occgrid <= 1)); % , find(map.occgrid == 0)
[jObs,iObs,kObs] = ind2sub(size(map.occgrid), find(map.occgrid == 1));
Qobs = [jObs, iObs,kObs];
Q = [j,i,k];
Q(getIndex(Qobs,Q),:) = inf;
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
front = cell(1);
front{1} = u;
old = cell(1);
b = inf(size(f));
% hold on;
while ~ismember(vg,old{1}) && M ~= inf
    front{1} = fast_setdiff(front{1},u);
    old{1} = [old{1};u];

    [nIdx, cost] = getNeighbors(Q,u,xSize,ySize,zSize,res);
    nIdx = fast_setdiff(nIdx,old{1});
    d = g(u) + cost(nIdx)';%ones(size(nIdx));
    lowIdx = d < g(nIdx); 
    g(nIdx(lowIdx)) = d(lowIdx);
    p(nIdx(lowIdx)) = u;
    
    front{1} = [front{1}; fast_setdiff(nIdx,front{1})];
    
    f = g + h;
%     f(nIdx(lowIdx)) = g(nIdx(lowIdx)) + h(nIdx(lowIdx));
    b(front{1}) = f(front{1});
    f(f~=b) = inf;
    uOld = u;
    [M,u] = min(f);
    b(front{1}) = inf;
    
    num_expanded = num_expanded + 1;
end

u = uOld;
pathIdx = cell(1);
while ~isnan(p(u))
    pathIdx{1} = [u;pathIdx{1}];
    u = p(u);
end

path = sub2pos(map,Q(pathIdx{1},:));
path = [start;path;goal];

end

%% helper functions
function ind = getIndex(v,Q)   
    [~,ind] = ismember(v,Q,'row');
end

function array = elementwise(a1,a2)
    array = a1.' + a2;
    array = array(:);
end

function [neighbors, cost] = getNeighbors(positions, index, xSize, ySize, zSize, res)
    if (positions(index,3) == zSize)
        add1 = [-xSize*ySize;0];
        cadd1 = [res(3);0].^2;
    elseif (positions(index,3) == 1)
        add1 = [0; xSize*ySize];
        cadd1 = [0;res(3)].^2;
    else
        add1= [-xSize*ySize;0;xSize*ySize];
        cadd1 = [res(3);0;res(3)].^2;
    end
    if (positions(index,2) == ySize)
        add2 = [-xSize;0];
        cadd2 = [res(2);0].^2;
    elseif (positions(index,2) == 1)
        add2 = [0; xSize];
        cadd2 = [0;res(2)].^2;
    else
        add2 = [-xSize;0;xSize];
        cadd2 = [res(2);0;res(2)].^2;
    end
    if (positions(index,1) == xSize)
        add3 = [-1;0];
        cadd3 = [res(1);0].^2;
    elseif (positions(index,1) == 1)
        add3 = [0; 1];
        cadd3 = [0;res(1)].^2;
    else
        add3 = [-1;0;1];
        cadd3 = [res(1);0;res(1)].^2;
    end

    neighbors = (index + elementwise(add3, elementwise(add2,add1)));
    cost(neighbors) = elementwise(cadd3, elementwise(cadd2,cadd1));
    neighbors(any(isinf(positions(neighbors(:,1),:)),2)) = [];
end

function C = fast_setdiff(A,B)
% MYSETDIFF Set difference of two sets of positive integers (much faster than built-in setdiff)
% C = my_setdiff(A,B)
% C = A \ B = { things in A that are not in B }
% 
if isempty(A) || isempty(B)
    C = A;
    return;
end

    check = false(1,max(max(A),max(B)));
    check(A) = true;
    check(B) = false;
    C = A(check(A));

end