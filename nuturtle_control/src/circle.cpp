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
#include "nuturtle_control/srv/circle.hpp"
#include "nuturtle_control/srv/reverse.hpp"
#include "nuturtle_control/srv/stop.hpp"

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
    circle_srv = create_service<nuturtle_control::srv::Circle>(
      "~/circle",
      std::bind(&Circle::circle_callback, this, std::placeholders::_1, std::placeholders::_2));
    reverse_srv = create_service<nuturtle_control::srv::Reverse>(
      "~/reverse",
      std::bind(&Circle::reverse_callback, this, std::placeholders::_1, std::placeholders::_2));
    stop_srv = create_service<nuturtle_control::srv::Stop>(
      "~/stop",
      std::bind(&Circle::stop_callback, this, std::placeholders::_1, std::placeholders::_2));
    timer =
      create_wall_timer(
      std::chrono::milliseconds(int(1.0 / frequency * 1000)),
      std::bind(&Circle::timer_callback, this)); // defaults to 100 Hz
  }

private:
  double frequency;
  State state;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
  rclcpp::Service<nuturtle_control::srv::Circle>::SharedPtr circle_srv;
  rclcpp::Service<nuturtle_control::srv::Reverse>::SharedPtr reverse_srv;
  rclcpp::Service<nuturtle_control::srv::Stop>::SharedPtr stop_srv;
  rclcpp::TimerBase::SharedPtr timer;
  geometry_msgs::msg::Twist zero_twist, circle_twist;

  /// \brief Timer callback
  void timer_callback()
  {
    if (state == State::STOP) {
    }
    if (state == State::GO) {
      cmd_vel_pub->publish(circle_twist);
    }
    if (state == State::END) {
      zero_twist.linear.x = 0.0;
      zero_twist.angular.z = 0.0;
      cmd_vel_pub->publish(zero_twist);
      state = State::STOP;
    }
  }

  /// \brief move the robot in a circle
  /// \param request - Circle.srv type, with double parameters radius and velocity
  /// \param response - Empty type
  void circle_callback(
    nuturtle_control::srv::Circle::Request::SharedPtr request,
    const nuturtle_control::srv::Circle::Response::SharedPtr)
  {
    circle_twist.angular.z = request->velocity;
    circle_twist.linear.x = request->radius * request->velocity;
    if (state != State::GO) {
      state = State::GO;
    }
  }

  /// \brief Reverse direction of robot
  /// \param request - Empty type
  /// \param response - Empty type
  void reverse_callback(
    const nuturtle_control::srv::Reverse::Request::SharedPtr,
    const nuturtle_control::srv::Reverse::Response::SharedPtr)
  {
    if (state != State::GO) {
      RCLCPP_INFO(get_logger(), "robot is not moving!");
    } else {
      RCLCPP_INFO(get_logger(), "reversing...");
    }
    circle_twist.angular.z = -circle_twist.angular.z;
    circle_twist.linear.x = -circle_twist.linear.x;
  }

  /// \brief Stop robot
  /// \param request - Empty type
  /// \param response - Empty type
  void stop_callback(
    const nuturtle_control::srv::Stop::Request::SharedPtr,
    const nuturtle_control::srv::Stop::Response::SharedPtr)
  {
    state = State::END;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Circle>());
  rclcpp::shutdown();
  return 0;
}
