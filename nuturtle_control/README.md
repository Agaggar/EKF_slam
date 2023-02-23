## turtle_control package - control robots

### Package Description
This package is designed to send control signals to the "red" (real) and "blue" (odometry) robots.
### Launchfile descriptions
`ros2 launch nuturtle_control start_robot.launch.xml` will launch the turtle_control and odometry nodes, along with rviz, circle, or teleop_twist_keyboard depending on the arguments. This launchfile also calls `load_one.launch.py` from the `nuturtle_description` package, and relies on `basic_world.yaml` and `diff_params.yaml` as well.

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
- `ros2 launch nuturtle_control start_robot.launch.xml cmd_src:='circle' robot:='localhost'` in ssh terminal
- `ros2 node list` in local computer terminal to ensure all nodes are running properly
    - it's most important that `/numsr_turtlebot` is running! If not, open a new ssh terminal and run `ros2 run numsr_turtlebot numsr_turtlebot`
- `ros2 service call /initial_pose turtle_control/srv/Teleport '{x: 0.0, y: 0.0, theta: 0.0}'`: initialize to (0,0,0) when starting on localhost. run on local computer terminal
- `ros2 service call /circle/circle turtle_control/srv/Circle '{velocity: 0.1, radius: 0.2}'` in local computer terminal to start turtle moving in a circle
- `ros2 service call /circle/stop turtle_control/srv/Stop` to stop turtlebot from local computer terminal
- `ros2 service call /circle/reverse turtle_control/srv/Reverse` to reverse CW/CCW direction from local computer terminal
- `ros2 topic echo /odom` to see odometry readings

### Video from Sample Call to See Odom Error
This video is sped up by a factor of 1/3.
<video src="https://user-images.githubusercontent.com/10903052/217920104-a421b8dc-50ce-4a4a-8b5e-cf925b39dd95.webm" data-canonical-src="https://user-images.githubusercontent.com/10903052/217920104-a421b8dc-50ce-4a4a-8b5e-cf925b39dd95.webm" controls="controls" muted="muted" class="d-block rounded-bottom-2 border-top width-fit" style="max-height:640px;">
</video>


### Odom Output from Above Video
header:
  stamp:
    sec: 1675970728
    nanosec: 469376419
  frame_id: odom
child_frame_id: blue/base_footprint
pose:
  pose:
    position:
      x: -0.03405921363741696
      y: 0.5003705671941315
      z: 0.0
    orientation:
      x: -0.0
      y: 0.0
      z: 0.9958111282727505
      w: -0.09143411184099538
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
