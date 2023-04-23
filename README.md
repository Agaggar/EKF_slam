# ME495 Sensing, Navigation and Machine Learning For Robotics
* Ayush Gaggar
* Winter 2023

Goal: This repository uses ROS2 in C++ to implement an extended kalman filter for a turtlebot3. Each subdirectory contains further detail, with the README in `nuslam` containing the final videos.

# Package List
This repository consists of several ROS packages
- nuturtle_description - basic package which creates a robot urdf and has the option to launch multiple robots in rviz in simulation; some launch files are later used by nusim
- nusim - simulator package that loads one robot and several obstacles in rviz
- turtlelib - not a package, but rather is a c++ library with transformation, vector, and twist definitions which will be useful later on
- nuturtle_control - contains nodes related to control of robot
- nuslam - implements feature-based extended kalman filter SLAM

Mostly worked by myself, with some questions answered by Ritika Ghosh, Ava Zahedi, James Oubre, and Nick Morales.
