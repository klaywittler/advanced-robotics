# advanced-robotics

Klayton Wittler - kwittler

Collaborators: Christopher Hsu

Project 2 Phase 1

init_script.m : loads the April Tag map, camera intrinsics, and transformation from imu to camera to send to estimate_pose.m
estimate_pose.m : estimates the pose of the robot utilizing known April Tag map and coordinates from an image by getting the homography between the 2 images and then estimates the rotation and translation from the homography using the fact that the April Tag map is planar. Converts origin of robot frame to camera using rotation and translation from parameters.txt then using estimated rotation and translation to convert to world frame. 

helper funcntions

estimate_homography.m : finds the homography between two frames using atleast 4 points
estimate_transformation.m : uses planar information and homography to derive the rotation and translation that results in homography
rot2quat.m : takes a rotation vector and converts to axis-angle representation and forms a unit quaternion from the axis-angle
aprilTagMap.m : creates 3D vector representing April Tag coordinates with convention of (x,y) position in first dimension, (p0,p1,p2,p3,p4) in second dimension, and (id) in 3 dimension
aprilTagMap.mat : saved April Tag coordinates in world frame to prevent recalculating every time

