/// \file
/// \brief control turtlebot
///
/// PARAMETERS:
///     wheel_radius (double): loaded from diff_params.yaml; wheel radius in m
///     track_width (double): loaded from diff_params.yaml; dist btwn wheels in m
///     motor_cmd_max (int): loaded from diff_params.yaml; max motor command input
///     motor_cmd_per_rad_sec (double): loaded from diff_params.yaml; motor command tick in rad/s
///     encoder_ticks_per_rad (double): loaded from diff_params.yaml; number of encoder ticks per radian
///     collision_radius (double): loaded from diff_params.yaml; collision radius in m
/// PUBLISHES:
///     wheel_cmd (nuturtlebot_msgs/WheelCommands): make turtlebot3 follow specified twist
///     joint_states (sensor_msgs/JointState): provide angle (rad) and vel (rad/sec)
/// SUBSCRIBES:
///     cmd_vel (geometry_msgs/Twist)
///     sensor_data (nuturtlebot_msgs/SensorData)
/// SERVERS:
///     TODO
/// CLIENTS:
///     TODO

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtle_control/msg/wheel_commands.hpp"
#include "turtle_control/msg/sensor_data.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "turtlelib/diff_drive.hpp"

using namespace std::chrono_literals;

class TurtleControl : public rclcpp::Node
{
public:
  TurtleControl()
  : Node("turtle_control"),
    wheel_radius(0.033),
    track_width(0.16), 
    motor_cmd_max(265),
    motor_cmd_per_rad_sec(0.024), 
    encoder_ticks_per_rad(651.8986),
    collision_radius(0.11)
  {
    // rcl_interfaces::msg::ParameterDescriptor wheel_radius_param_desc;
    // wheel_radius_param_desc.name = "wheel_radius";
    // wheel_radius_param_desc.type = 3;         // rate is a double
    // wheel_radius_param_desc.description = "simulation refresh rate (hz)";
    // // rate defaults to 200.0
    declare_parameter("wheel_radius", rclcpp::ParameterValue(-1.0));
    declare_parameter("track_width", rclcpp::ParameterValue(-1.0));
    declare_parameter("motor_cmd_max", rclcpp::ParameterValue(-1));
    declare_parameter("motor_cmd_per_rad_sec", rclcpp::ParameterValue(-1.0));
    declare_parameter("encoder_ticks_per_rad", rclcpp::ParameterValue(-1.0));
    declare_parameter("collision_radius", rclcpp::ParameterValue(-1.0));
    get_parameter("wheel_radius", wheel_radius);
    get_parameter("track_width", track_width);
    get_parameter("motor_cmd_max", motor_cmd_max);
    get_parameter("motor_cmd_per_rad_sec", motor_cmd_per_rad_sec);
    get_parameter("encoder_ticks_per_rad", encoder_ticks_per_rad);
    get_parameter("collision_radius", collision_radius);
    std::vector<double> params_set {wheel_radius, track_width, double(motor_cmd_max),
                                    motor_cmd_per_rad_sec, encoder_ticks_per_rad, collision_radius};

    for (unsigned int i = 0; i < params_set.size(); i++) {
      if (params_set.at(i) == -1.0) {
        RCLCPP_ERROR(get_logger(), "One or more parameters not set!");
        rclcpp::shutdown();
      }
    }

    wheel_cmd_pub = create_publisher<turtle_control::msg::WheelCommands>("~/wheel_cmd", 10);
    js_pub = create_publisher<sensor_msgs::msg::JointState>("~/joint_states", 10);
    cmd_vel_sub = create_subscription<geometry_msgs::msg::Twist>("~/cmd_vel", 10, std::bind(&TurtleControl::cmd_vel_callback, this, std::placeholders::_1));
    sensor_data_sub = create_subscription<turtle_control::msg::SensorData>("~/sensor_data", 10, std::bind(&TurtleControl::sd_callback, this, std::placeholders::_1));
    timer =
      create_wall_timer(
      std::chrono::milliseconds(int(1.0 / 200.0 * 1000)),
      std::bind(&TurtleControl::timer_callback, this)); // timer at 200 Hz
  }

private:
  double wheel_radius, track_width;
  int motor_cmd_max;
  double motor_cmd_per_rad_sec, encoder_ticks_per_rad, collision_radius;
  rclcpp::Publisher<turtle_control::msg::WheelCommands>::SharedPtr wheel_cmd_pub;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
  rclcpp::Subscription<turtle_control::msg::SensorData>::SharedPtr sensor_data_sub;
  rclcpp::TimerBase::SharedPtr timer;
  // more stuff
  // inc timer callback

  /// \brief Timer callback
  void timer_callback()
  {
     RCLCPP_INFO(get_logger(), "node works");
  }

  /// \brief cmd_vel subscription callback
  void cmd_vel_callback(const geometry_msgs::msg::Twist &twist) {
    ; // do stuff
  }

  /// \brief sensor_data subscription callback
  void sd_callback(const turtle_control::msg::SensorData) {
    ; //do stuff
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleControl>());
  rclcpp::shutdown();
  return 0;
}
