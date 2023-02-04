/// \file
/// \brief publishes odometry messages and transform
///
/// PARAMETERS:
///     body_id (string): The name of the body frame of the robot
///     odom_id (string): The name of the odometry frame
///     wheel_left (string): The name of the left wheel joint
///     wheel_right (string): The name of the right wheel joint
/// PUBLISHES:
///     TODO
/// SUBSCRIBES:
///     joint_states (sensor_msgs/JointState): update internal odometry
/// SERVERS:
///     initial_pose (nusim::srv::Teleport): reset odometry to a specified x, y, theta positions
/// CLIENTS:
///     none
/// http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "turtle_control/srv/teleport.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class Odometry : public rclcpp::Node
{
public:
  Odometry()
  : Node("odometry"),
    body_id(),
    odom_id("odom"),
    wheel_left(),
    wheel_right()
  {
    declare_parameter("body_id", rclcpp::ParameterValue(std::to_string(-1.0)));
    declare_parameter("odom_id", rclcpp::ParameterValue("odom"));
    declare_parameter("wheel_left", rclcpp::ParameterValue(std::to_string(-1.0)));
    declare_parameter("wheel_right", rclcpp::ParameterValue(std::to_string(-1.0)));
    get_parameter("body_id", body_id);
    get_parameter("odom_id", odom_id);
    get_parameter("wheel_left", wheel_left);
    get_parameter("wheel_right", wheel_right);
    std::vector<std::string> params_set {body_id, wheel_left, wheel_right};
    for (unsigned int i = 0; i < params_set.size(); i++) {
        if (params_set.at(i) == std::to_string(-1.0)) {
            RCLCPP_ERROR(get_logger(), "One or more parameters not set!");
            rclcpp::shutdown();
        }
    }
    tf2_rostf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    js_sub = create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&Odometry::js_callback, this, std::placeholders::_1));
    initial_pose_srv = create_service<turtle_control::srv::Teleport>(
      "/initial_pose",
      std::bind(&Odometry::ip_srv_callback, this, std::placeholders::_1, std::placeholders::_2));
    timer =
    create_wall_timer(
        std::chrono::milliseconds(int(1.0 / 200.0 * 1000)),
        std::bind(&Odometry::timer_callback, this)); // timer at 200 Hz
  }

private:
  std::string body_id, odom_id, wheel_left, wheel_right;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf2_rostf_broadcaster;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub;
  rclcpp::Service<turtle_control::srv::Teleport>::SharedPtr initial_pose_srv;
  rclcpp::TimerBase::SharedPtr timer;

  std::vector<double> config{0.0, 0.0, 0.0}; // x, y, theta

  /// \brief Timer callback
  void timer_callback()
  {
    // current_time = get_clock()->now();
    RCLCPP_INFO(get_logger(), "node works");
  }

  void js_callback(const sensor_msgs::msg::JointState js) {
    ; // do stuff
  }

  void ip_srv_callback(turtle_control::srv::Teleport::Request::SharedPtr request,
                       turtle_control::srv::Teleport::Response::SharedPtr response) {
    RCLCPP_INFO(get_logger(), "Teleport service...");
    config.at(0) = request->x;
    config.at(1) = request->y;
    config.at(2) = request->theta;
  }

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometry>());
  rclcpp::shutdown();
  return 0;
}
