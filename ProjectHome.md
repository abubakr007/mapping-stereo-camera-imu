This was my scientific project which it's objective was to create a 3D map from an environment using a regular stereo camera and a IMU (Inertial Measurement Unit).
The language used is C/C++, using the libraries OpenCV, Octomap and ROS.
Three ROS packages was created to accomplish the final mapping.
The package IMUSensor is responsible to capture the IMU's informations, like pitch, roll and yaw.
The package StereoCapture is responsible to capture the Stereo Camera's informations, like the left and right colored images.
The package StereoVision is responsible to calculte the disparity map from the stereo pair and, from the disparity map, create the point cloud with the real distance from the camera to any point.
The package Mapping is responsible to read the point cloud and the IMU's informations to create the 3D map from the environment.