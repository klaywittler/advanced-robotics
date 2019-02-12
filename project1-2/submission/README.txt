# advanced-robotics

Klayton Wittler - kwittler

Collaborators: Christopher Hsu, Luca Scheuer

Project 1 Phase 2

dijkstra.m : runs both Dijkstra and Astar where the Astar heuristic is Euclidean distance to the goal position. The algorithm utilizes a set of vertices on the "frontier" to be explored and set of already "explored vertices which are indicated by logical arrays of where a vertex belongs or no. Each vertex of the space is assigned an index and the index is what is tracked throughout the algorithm to prevent point comparisons. The parent of each node is tracked as the cheapest way to get to that node and the path to the goal is retrieved by tracing back the parents to the starting node. The get the neighbors of a node, a pattern in the assignment of vertices is used to index one unit all around.