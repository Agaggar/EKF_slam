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

