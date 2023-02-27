/// \file
/// \brief publishes odometry messages and transform
///
/// PARAMETERS:
///     body_id (string): The name of the body frame of the robot
///     odom_id (string): The name of the odometry frame
///     wheel_left (string): The name of the left wheel joint
///     wheel_right (string): The name of the right wheel joint
/// PUBLISHES:
///     odom (nav_msgs/msg/Odometry): odometry message
/// SUBSCRIBES:
///     joint_states (sensor_msgs/JointState): update internal odometry
/// SERVERS:
///     initial_pose (nusim::srv::Teleport): reset odometry to a specified x, y, theta positions
/// CLIENTS:
///     none

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nusim/srv/teleport.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
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
    wheel_right(),
    timestep(0),
    pub_odom(true)
  {
    declare_parameter("body_id", rclcpp::ParameterValue(std::to_string(-1.0)));
    declare_parameter("odom_id", rclcpp::ParameterValue("odom"));
    declare_parameter("wheel_left", rclcpp::ParameterValue(std::to_string(-1.0)));
    declare_parameter("wheel_right", rclcpp::ParameterValue(std::to_string(-1.0)));
    get_parameter("body_id", body_id);
    get_parameter("odom_id", odom_id);
    get_parameter("wheel_left", wheel_left);
    get_parameter("wheel_right", wheel_right);
    declare_parameter("pub_odom", rclcpp::ParameterValue(pub_odom));
    get_parameter("pub_odom", pub_odom);
    std::vector<std::string> params_set {body_id, wheel_left, wheel_right};

    // better to check the return value from get_parameter rather than setting a default string value
    for (unsigned int i = 0; i < params_set.size(); i++) {
      if (params_set.at(i) == std::to_string(-1.0)) {
        RCLCPP_ERROR(get_logger(), "One or more parameters not set!");
        rclcpp::shutdown();
      }
    }
    tf2_rostf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    odom_pub = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    js_sub = create_subscription<sensor_msgs::msg::JointState>(
      "/blue/joint_states",
      10, std::bind(&Odometry::js_callback, this, std::placeholders::_1));
    initial_pose_srv = create_service<nusim::srv::Teleport>(
      "/initial_pose",
      std::bind(&Odometry::ip_srv_callback, this, std::placeholders::_1, std::placeholders::_2));
    blue_path_pub = create_publisher<nav_msgs::msg::Path>("~/bluepath", 10);
    timer =
      create_wall_timer(
      std::chrono::milliseconds(int(1.0 / 200.0 * 1000)),
      std::bind(&Odometry::timer_callback, this));   // timer at 200 Hz
  }

private:
  std::string body_id, odom_id, wheel_left, wheel_right;
  size_t timestep;
  bool pub_odom;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf2_rostf_broadcaster;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr initial_pose_srv;
  rclcpp::TimerBase::SharedPtr timer;
  turtlelib::DiffDrive nubot = turtlelib::DiffDrive(0.0, 0.0, 0.0, 0.0, 0.0);
  sensor_msgs::msg::JointState js_msg;
  std::vector<double> config{0.0, 0.0, 0.0}; // x, y, theta
  geometry_msgs::msg::TransformStamped t;
  nav_msgs::msg::Path blue_path;
  geometry_msgs::msg::PoseStamped current_point;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr blue_path_pub;

  /// \brief Timer callback
  void timer_callback()
  {
    if (timestep == 0) {
      t.header.frame_id = odom_id;
      t.child_frame_id = body_id;
      t.transform.translation.z = 0.0;
      blue_path.header.frame_id = "nusim/world";
      current_point.header.frame_id = "nusim/world";
      current_point.pose.position.z = 0.0;
    }
    t.header.stamp = get_clock()->now();
    t.transform.translation.x = nubot.getCurrentConfig().at(0);
    t.transform.translation.y = nubot.getCurrentConfig().at(1);
    tf2::Quaternion q;
    q.setRPY(0, 0, nubot.getCurrentConfig().at(2));
    q.normalize();
    geometry_msgs::msg::Quaternion q_geom = tf2::toMsg(q);
    t.transform.rotation = q_geom;

    if (timestep % 50 == 0) {
      blue_path.header.stamp = get_clock()->now();
      current_point.header.stamp = get_clock()->now();
      current_point.pose.position.x = nubot.getCurrentConfig().at(0);
      current_point.pose.position.y = nubot.getCurrentConfig().at(1);
      blue_path.poses.push_back(current_point);
      blue_path_pub->publish(blue_path);
    }

    tf2_rostf_broadcaster->sendTransform(t);
    if (pub_odom) {
      odom_pub->publish(compute_odom());
    }
  }

  /// \brief callback function for /joint_states subscription  
  /// \param js - JointState message received 
  void js_callback(const sensor_msgs::msg::JointState js)
  {
    if ((js_msg.velocity.size() > 1)) {
      nubot.fkinematics(
        std::vector<double>{js.position.at(0) - js_msg.position.at(0),
          js.position.at(1) - js_msg.position.at(1)});
    }
    js_msg = js;
  }

  /// \brief initial_pose service callback
  /// \param request - nuturtle_control teleport type
  void ip_srv_callback(
    nusim::srv::Teleport::Request::SharedPtr request,
    nusim::srv::Teleport::Response::SharedPtr)
  {
    RCLCPP_INFO(get_logger(), "Initial pose service...");
    nubot.setCurrentConfig(std::vector<double>{request->x, request->y, request->theta});
    config.at(0) = request->x;
    config.at(1) = request->y;
    config.at(2) = request->theta;
  }

  /// \brief helper function to compute and return an odometry message
  /// \return odometry message
  nav_msgs::msg::Odometry compute_odom()
  {
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
