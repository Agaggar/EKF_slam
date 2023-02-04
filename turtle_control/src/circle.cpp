/// \file
/// \brief publishes cmd_vel to move robot in a circle
///
/// PARAMETERS:
///     frequency (double): freq of cmd_vel commands to be published
/// PUBLISHES:
///     cmd_vel (geometry_msgs/msg/Twist): command velocity
/// SUBSCRIBES:
///     none
/// SERVERS:
///     control (#todo: custom srv type): velocity and radius of arc
///     reverse (std_srvs/Empty): reverse direction of robot
///     stop (std_srvs/Empty): stops robot
/// CLIENTS:
///     none

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"

using namespace std::chrono_literals;


enum class State {STOP, GO, END};

class Circle : public rclcpp::Node
{
public:
  Circle()
  : Node("circle"),
    frequency(100.0),
    state(State::STOP)
  {
    declare_parameter("frequency", rclcpp::ParameterValue(frequency));
    get_parameter("frequency", frequency);
    cmd_vel_pub = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    timer =
      create_wall_timer(
      std::chrono::milliseconds(int(1.0 / frequency * 1000)),
      std::bind(&Circle::timer_callback, this)); // defaults to 100 Hz
  }

private:
  double frequency;
  State state;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
  rclcpp::TimerBase::SharedPtr timer;
  geometry_msgs::msg::Twist zero_twist{geometry_msgs::msg::Vector3{0.0, 0.0, 0.0}, geometry_msgs::msg::Vector3{0.0, 0.0, 0.0}};
  geometry_msgs::msg::Twist circle_twist;
  double radius, velocity;

  /// \brief Timer callback
  void timer_callback()
  {
    if (state == State::GO) {
        // pub to cmd_vel
    }
    if (state == State::END) {
        cmd_vel_pub->publish(zero_twist);
    }
    // current_time = get_clock()->now();
    // RCLCPP_INFO(get_logger(), "node works");
  }

  /// \brief move the robot in a circle
  /// \param request - custom srv type, with double parameters radius and velocity
  /// \param response - Empty type
  void reverse(
    const std_srvs::srv::Empty::Request::SharedPtr request,
    const std_srvs::srv::Empty::Response::SharedPtr response)
  {
  }

  /// \brief Reverse direction of robot
  /// \param request - Empty type
  /// \param response - Empty type
  void reverse(
    const std_srvs::srv::Empty::Request::SharedPtr request,
    const std_srvs::srv::Empty::Response::SharedPtr response)
  {
  }
};
  
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometry>());
  rclcpp::shutdown();
  return 0;
}
