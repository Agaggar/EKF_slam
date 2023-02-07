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
/// ros2 run turtle_control odometry --ros-args -p body_id:="hi" -p odom_id:="hi" -p wheel_left:="left" -p wheel_right:="right"

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "turtle_control/srv/teleport.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "turtlelib/diff_drive.hpp"

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
    odom_pub = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    js_sub = create_subscription<sensor_msgs::msg::JointState>("/blue/joint_states", 10, std::bind(&Odometry::js_callback, this, std::placeholders::_1));
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
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub;
  rclcpp::Service<turtle_control::srv::Teleport>::SharedPtr initial_pose_srv;
  rclcpp::TimerBase::SharedPtr timer;
  turtlelib::DiffDrive nubot = turtlelib::DiffDrive(0.0, 0.0, 0.0, 0.0, 0.0);
  sensor_msgs::msg::JointState js_msg; // = sensor_msgs::msg::JointState();
  // js_msg.header.stamp = get_clock()->now();
  // js_msg.name = std::vector<std::string>{"left wheel, right wheel"};
  // js_msg.position = std::vector<double>{0.0, 0.0}; // assuming robot starts with wheels at 0, 0 
  // js_msg.velocity = std::vector<double>{0.0, 0.0}; // assuming robot starts at rest 
  std::vector<double> config{0.0, 0.0, 0.0}; // x, y, theta
  geometry_msgs::msg::TransformStamped t;
  // bool sd_received{false};

  /// \brief Timer callback
  void timer_callback()
  {
    t.header.stamp = get_clock()->now();
    t.header.frame_id = odom_id;
    t.child_frame_id = body_id;
    t.transform.translation.x = nubot.getCurrentConfig().at(0);
    t.transform.translation.y = nubot.getCurrentConfig().at(1);
    t.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, nubot.getCurrentConfig().at(2));
    q.normalize();
    geometry_msgs::msg::Quaternion q_geom = tf2::toMsg(q);
    t.transform.rotation = q_geom;
    // RCLCPP_INFO(get_logger(), std::cout << t);
    tf2_rostf_broadcaster->sendTransform(t);
    odom_pub->publish(compute_odom());
    // if (js_msg.velocity.size() > 0) {
    //   odom_pub->publish(compute_odom());
    // }
  }

  void js_callback(const sensor_msgs::msg::JointState js) {
    if ((js_msg.velocity.size() > 0)) {
      nubot.fkinematics(std::vector<double>{js.position.at(0) - js_msg.position.at(0),
                                            js.position.at(1) - js_msg.position.at(1)});
      // sd_received = true;
    }
    js_msg = js;
  }

  void ip_srv_callback(turtle_control::srv::Teleport::Request::SharedPtr request,
                       turtle_control::srv::Teleport::Response::SharedPtr) {
    RCLCPP_INFO(get_logger(), "Teleport service...");
    nubot.setCurrentConfig(std::vector<double>{request->x, request->y, request->theta});
    config.at(0) = request->x;
    config.at(1) = request->y;
    config.at(2) = request->theta;
  }

  nav_msgs::msg::Odometry compute_odom() {
    nav_msgs::msg::Odometry odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = get_clock()->now();
    odom_msg.header.frame_id = odom_id;
    odom_msg.child_frame_id = body_id;
    odom_msg.pose.pose.position.x = nubot.getCurrentConfig().at(0);
    odom_msg.pose.pose.position.y = nubot.getCurrentConfig().at(1);
    odom_msg.pose.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, nubot.getCurrentConfig().at(2));
    q.normalize();
    geometry_msgs::msg::Quaternion q_geom = tf2::toMsg(q);
    odom_msg.pose.pose.orientation = q_geom;
    std::array<double, 36> cov;
    cov.fill(0.0);
    odom_msg.pose.covariance = cov;
    turtlelib::Twist2D vb{0.0, 0.0, 0.0};
    if ((js_msg.velocity.size() > 0)) {
      vb = nubot.velToTwist(js_msg.velocity);
    }
    odom_msg.twist.twist.linear.x = vb.linearx;
    odom_msg.twist.twist.linear.y = vb.lineary;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = vb.angular;
    odom_msg.twist.covariance = cov;
    return odom_msg;
  }

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometry>());
  rclcpp::shutdown();
  return 0;
}
