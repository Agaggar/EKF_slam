## nusim package - the simulator

### Package Description
This package is designed to launch a single robot with a set number of obstacles in a simulation space. The coordinates and orientation of the robot and the coordinates and number of obstacles can be defined in `config/basic_world.yaml`.

### Launchfile descriptions
`ros2 launch nusim nusim.launch.xml` will launch nusim, rviz, and a robot and joint state publisher. Thhis launch file calls `load_one.launch.py` from nuturtle_description.

### Parameters
Parameters can only be changed in `config/basic_world.yaml`. The parameters include:

- x0: initial x coordinate of the robot, m
- y0: initial y coordinate of the robot, m
- z0: initial z coordinate of the robot, m
- theta0: initial angle heading of the robot, rad
- obstacles.x: a list of the x-coordinates of obstacles
- obstacles.y: a list of the y-coordinates of obstacles
- obstacles.r: radius of obstacles

### Services Sample Calls 
- `ros2 service call /nusim/teleport nusim/srv/Teleport '{x: 1.0, y: 2.0, theta: 0.0}'`: teleports the robot to the x, y, and theta values specified
- `ros2 service call /nusim/reset std_srvs/srv/Empty`: reset the timestep value

### Rviz screenshot
![Rviz_screenshot](https://github.com/ME495-Navigation/nuturtle-Agaggar/blob/hw1/TaskC/nusim/images/nusim1.png?raw=true)