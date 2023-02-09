## turtle_control package - control robots

### Package Description
This package is designed to send control signals to the "red" (real) and "blue" (odometry) robots.
### Launchfile descriptions
`ros2 launch turtle_control start_robot.launch.xml` will launch the turtle_control and odometry nodes, along with rviz, circle, or teleop_twist_keyboard depending on the arguments. This launchfile also calls `load_one.launch.py` from the `nuturtle_description` package, and relies on `basic_world.yaml` and `diff_params.yaml` as well.

### Parameters
Parameters can be changed in `basic_world.yaml` and `diff_params.yaml`. See the `nusim` and `nuturtle_description` packages respectively for more more information.

A list of parameters in the launch file are as follow:
- cmd_src: Choices for control input: ['circle', 'teleop', 'none']; defaults to circle.
- robot: Choices for the robot: ['nusim', 'localhost', 'none']; defaults to nusim (simulator).
- use_rviz: Choices to use rviz: ['true', 'false']; (declared in start_robot, used in load_one launch)

### Services Sample Calls 
- `ros2 service call /circle/circle turtle_control/srv/Circle '{velocity: 2.0, radius: 1.0}'`: drive the robot in a circle with velocity and radius specified
- `ros2 service call /circle/stop turtle_control/srv/Stop`: stop moving the robot
- `ros2 service call /circle/reverse turtle_control/srv/Reverse`: reverse the motion of the robot

### Sample Call in Localhost
- `./aarch64 colcon_aarch64` to cross compile on your local computer terminal
- make sure the turtlebot and your computer are connected to the same wifi network
- `rsync -av --delete aarch64_install/ msr@<turtlebot_name>:/home/msr/install` to send to turtlebot
- in a new terminal, `ssh -oSendEnv=ROS_DOMAIN_ID msr@<turtlebot_name>`
- `source install/setup.bash` in ssh terminal
- `ros2 launch start_robot.launch.xml cmd_src:='circle' robot:='localhost'` in ssh terminal
- `ros2 node list` in local computer terminal to ensure all nodes are running properly
    - it's most important that `/numsr_turtlebot` is running! If not, open a new ssh terminal and run `ros2 run numsr_turtlebot numsr_turtlebot`
- `ros2 service call /initial_pose turtle_control/srv/Teleport '{x: 0.0, y: 0.0, theta: 0.0}'`: initialize to (0,0,0) when starting on localhost. run on local computer terminal
- `ros2 service call /circle/circle turtle_control/srv/Circle '{velocity: 0.1, radius: 0.2}'` in local computer terminal to start turtle moving in a circle
- `ros2 service call /circle/stop turtle_control/srv/Stop` to stop turtlebot from local computer terminal
- `ros2 service call /circle/reverse turtle_control/srv/Reverse` to reverse CW/CCW direction from local computer terminal
- `ros2 topic echo /odom` to see odometry readings

### Odom Output from Above Video
header:
  stamp:
    sec: 1675956490
    nanosec: 828922108
  frame_id: odom
child_frame_id: blue/base_footprint
pose:
  pose:
    position:
      x: -0.1706329929386896
      y: 1.489721926258149
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: -0.9946480890625745
      w: 0.10332075746997196
  covariance:
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
twist:
  twist:
    linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
  covariance:
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
---