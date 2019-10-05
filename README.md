# advanced-robotics

## Sections

* [Project 1](#project-1)
    * [Phase 1: Controller](#phase-1-controller)
    * [Phase 2: Path Planning](#phase-2-path-planning)
    * [Phase 3: Optimizations](#phase-3-optimizations)
* [Project 2](#project-2)
    * [Phase 1: Visual Pose Estimation](#phase-1-visual-pose-estimation)
    * [Phase 2: Visual Velocity Estimation](#phase-2-visual-velocity-estimation)
    * [Phase 3: EKF State Estimation](#phase-3-ekf)

## Project 1 

### Phase 1: Controller

Nonlinear geometric controller with generated trajectories of a diamond and helix utlizing quadratic velocity profiles.

More can be found in the project1-1 [submission](project1-1/submission) folder.
![](images/project1_phase1_circleTraj.png)

### Phase 2: Path Planning

Builds upon project1-1 by running either Dijkstra or Astar with Euclidean distance heuristic to plan a path to goal position.

More can be found in the project1-2 [submission](project1-2/submission) folder
![](images/project1_phase2_pathplan.png)

### Phase 3: Optimizations

Runs project1-2 with additional pruning of waypoints through ray tracing to make trajectory generation more efficient and fix discretization. Also creates polynomial trajectories by solving a linear system between waypoints to smoothly travel to the goal.

More can be found in the project1-3 [submission](project1-3/submission) folder
![](images/project1_phase3.png)

## Project 2

### Phase 1: Visual Pose Estimation

Uses April Tag information, camera intrinsics, and transformations from IMU to estimate pose between two images with homographies.

More can be found in the project2-1 [submission](project2-1/submission) folder
![](images/project2_phase1.png)

### Phase 2: Visual Velocity Estimation

Builds on project2-1 by utlizing optical flow between two images and RANSAC to estimate velocities.

More can be found in the project2-2 [submission](project2-2/submission) folder
![](images/project2_phase2.png)

### Phase 3: EKF State Estimation

Incorporates the dynamics of the quadrotor and an IMU in with the velocity and pose estimates from project2-2 to use an extended kalman filter (ekf).

More can be found in the project2-3 [submission](project2-3/submission) folder
![](images/project2_phase3.png)
