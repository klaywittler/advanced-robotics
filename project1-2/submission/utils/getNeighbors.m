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

%%%%%%%%%%%%%%%%%%%%%%%
%%% helper function %%%
%%%%%%%%%%%%%%%%%%%%%%%
function array = elementwise(a1,a2)
% elementwise convoles the two arrays
% parameters:
%   a1 - 1xn vetor
%   a2 - 1xn vector
    array = a1.' + a2;
    array = array(:);
end