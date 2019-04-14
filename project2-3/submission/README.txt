# advanced-robotics

Klayton Wittler - kwittler

Collaborators: Christopher Hsu, Luca Scheuer

Project 2 Phase 3

init_script.m : loads the April Tag map, camera instrinsic, and transformation from imu to camera to send to estimate_pose.m
ekf1.m : 
ekf2.m : 

helper funcntions

getParameters1.m : creates linear model as well as returns nonlinear state derivative
getParameters2.m : creates linear model as well as returns nonlinear state derivative

prediction() : utilizes previous state and inputs to propagate the dynmaics forward one time step
measurement() : updates state given a set of measurements

estimate_state.m : estimates the velocity and angular velocity of the robot utilizing optical flow and RANSAC. Features in the image are found with detectFASTFeatures(I) and tracked. The previous image is stored to find the pixel velocities (optical flow) then 	RANSAC is used to find the best estimate of velocity that explains the optical flow. Also calls estimate_pose to return position and orientation

estimate_pose.m : estimates the pose of the robot utilizing known April Tag map and coordinates from an image by getting the homography between the 2 images and then estimates the rotation and translation from the homography using the fact that the April Tag map is planar. Converts origin of robot frame to camera using rotation and translation from parameters.txt then using estimated rotation and translation to convert to world frame. 

	helper functions
	estimate_homography.m : finds the homography between two frames using atleast 4 points
	estimate_transformation.m : uses planar information and homography to derive the rotation and translation

aprilTagMap.m : creates 3D vector representing April Tag coordinates with convention of (x,y) position in first dimension, 	(p0,p1,p2,p3,p4) in second dimension, and (id) in 3 dimension
aprilTagMap.mat : saved April Tag coordinates in world frame to prevent recalculating every time

motionRANSAC.m : performs 80 iteration with a sample size of 3 to form a square motion field equation in finding a estimated velocity. 	The etimated velocity is then applied to the rest of the points to see if the optical flows match. The L2 norm squared is used 	as the error metric when comparing the optical flows with a threshold of 0.0005 for considering an inlier.
	
	helper functions
	
	getAB() : constructs the motion field equation for all the points in a 3D matrix - index in first dimension, (x,y) in second 		dimension, and velocity and omega in third dimension
	getDepth() : finds the depth for all the points utilizing the homography, planar information, rotation and translation
	estimate_velocity : returns the velocity for a set of points
	error() : returns the L2 norm squared as an error metric

reference: CIS 580 - Machine Perception




