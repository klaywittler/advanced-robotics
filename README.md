# advanced-robotics

Klayton Wittler - kwittler

Project 1 Phase 1

controller.m : Implemented geometric nonlinear controller

diamond.m : trajectory is the concatentation of independent quadratic desired velcities from point to point so it reaches a stop before going to the next point. The position is found from integrating the desired velocity using intial conditions to find necessary constants and the acceleration is found from taking its derivative.

circle.m : trajectory is a time parameterized angle which is velocity is set up to speed up and slow back down half way through the helix. With the time parameterized angle, the integral and derivative can be taken to obtain angular position and acceleration. This angle then forms the basis for the quadrotors position along the helix.

Project 1 Phase 2

dijkstra.m : runs both Dijkstra and Astar where the Astar heuristic is Euclidean distance to the goal position. The algorithm utilizes a set of vertices on the "frontier" to be explored and set of already "explored vertices which are indicated by logical arrays of where a vertex belongs or no. Each vertex of the space is assigned an index and the index is what is tracked throughout the algorithm to prevent point comparisons. The parent of each node is tracked as the cheapest way to get to that node and the path to the goal is retrieved by tracing back the parents to the starting node. The get the neighbors of a node, a pattern in the assignment of vertices is used to index one unit all around.

helper funcntions in dijkstra.m

getIndex: returns index of an element in a matrix

/utils helper functions:

getNeighbors.m : finds the indices for all vertices that neighbors one of interest
elementwise : helps in convoling all possibilities of directions of travel
