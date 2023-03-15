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
