## nuslam package - contains node for extended kalman filter slam

### Package Description
This package is designed to do ekf slam to correct the position discrepancy between odometry and the real robot

### Launchfile descriptions
* simply run `ros2 launch nuslam slam.launch.xml cmd_src:='teleop'` to launch the slam node, along with necessary rviz files and the turtlebot teleop keyboard to control the redbot in rviz
* run `ros2 launch nuslam landmark_detect.launch.xml cmd_src:='teleop' rviz_landmarks:='true'` to launch with landmark clustering
* run `ros2 launch nuslam unknown_data_assoc.launch.xml cmd_src:='teleop'` to launch with data association

### Parameters
* Parameters can be changed in `basic_world.yaml` and `diff_params.yaml`. See the `nusim` and `nuturtle_description` packages respectively for more more information. Addtionally, there are labeled additional parameters for sensor noise, slip fraction, max_range of the sensor, etc.
* all the same parameter descriptions from nuturtle-control package can be used here

## Final RViZ screenshot:
![rviz_slam](https://user-images.githubusercontent.com/10903052/225189389-1a9ffe9d-fe31-4e36-bcae-2ee362ce24d6.png)

## Final RViZ Video (for reference):
![rviz video](https://user-images.githubusercontent.com/10903052/225190815-6669c641-a22d-4db9-ae6b-6331ea5b974b.webm)


### Sample Call in Localhost
- `./aarch64 colcon_aarch64` to cross compile on your local computer terminal
- make sure the turtlebot and your computer are connected to the same wifi network
- `rsync -av --delete aarch64_install/ msr@<turtlebot_name>:/home/msr/install` to send to turtlebot
- in a new terminal, `ssh -oSendEnv=ROS_DOMAIN_ID msr@<turtlebot_name>`
- `source install/setup.bash` in ssh terminal
- `ros2 launch nuslam turtlebot_bringup.launch.xml`



- `ros2 launch nuturtle_control start_robot.launch.xml cmd_src:='circle' robot:='localhost'` in ssh terminal
- `ros2 node list` in local computer terminal to ensure all nodes are running properly
    - it's most important that `/numsr_turtlebot` is running! If not, open a new ssh terminal and run `ros2 run numsr_turtlebot numsr_turtlebot`
- `ros2 service call /initial_pose turtle_control/srv/Teleport '{x: 0.0, y: 0.0, theta: 0.0}'`: initialize to (0,0,0) when starting on localhost. run on local computer terminal
- `ros2 service call /circle/circle turtle_control/srv/Circle '{velocity: 0.1, radius: 0.2}'` in local computer terminal to start turtle moving in a circle
- `ros2 service call /circle/stop turtle_control/srv/Stop` to stop turtlebot from local computer terminal
- `ros2 service call /circle/reverse turtle_control/srv/Reverse` to reverse CW/CCW direction from local computer terminal
- `ros2 topic echo /odom` to see odometry readings