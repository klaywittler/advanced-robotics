# advanced-robotics

Klayton Wittler - klaywittler

Project 1 Phase 1

controller.m : Implemented geometric nonlinear controller

diamond.m : trajectory is the concatentation of independent quadratic desired velcities from point to point so it reaches a stop before going to the next point. The position is found from integrating the desired velocity using intial conditions to find necessary constants and the acceleration is found from taking its derivative.

circle.m : trajectory is time parameterization of a circle for x and y, with a quadratic desired velocity in the z direction and is integrated to find the desired postion and its derivative is the desired acceleration. The quadratic desired velocity starts at 0 and ends at 0 with a maximum half way through the trajectory.
