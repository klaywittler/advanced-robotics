# advanced-robotics

Klayton Wittler - kwittler

Project 1 Phase 1

controller.m : Implemented geometric nonlinear controller

diamond.m : trajectory is the concatentation of independent quadratic desired velcities from point to point so it reaches a stop before going to the next point. The position is found from integrating the desired velocity using intial conditions to find necessary constants and the acceleration is found from taking its derivative.

circle.m : trajectory is a time parameterized angle which is velocity is set up to speed up and slow back down half way through the helix. With the time parameterized angle, the integral and derivative can be taken to obtain angular position and acceleration. This angle then forms the basis for the quadrotors position along the helix.
