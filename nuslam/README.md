## nuslam package - contains node for extended kalman filter slam

### Package Description
This package is designed to do ekf slam to correct the position discrepancy between odometry and the real robot

### Launchfile descriptions
* simply run `ros2 launch nuslam slam.launch.xml cmd_src:='teleop'` to launch the slam node, along with necessary rviz files and the turtlebot teleop keyboard to control the redbot in rviz

### Parameters
* Parameters can be changed in `basic_world.yaml` and `diff_params.yaml`. See the `nusim` and `nuturtle_description` packages respectively for more more information. Addtionally, there are labeled additional parameters for sensor noise, slip fraction, max_range of the snesor, etc.
* all the same parameter descriptions from nuturtle-control package can be used here

## Final RViZ screenshot:


## Note:
This package isn't fully functional. 
* Specifically, collisions between the red robot and obstacles isn't fully implemented (requires a delay to recognize obstacles and react due to frequency discrepancy between fake_sensor and cmd_vel)
* SLAM still follows odometry rather than updating properly
* Landmark detection in SLAM does unknown things when the robot is rotating, with landmarks frequently deviating in the y direction