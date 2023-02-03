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
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "turtle_control/msg/wheel_commands.hpp"
#include "turtle_control/msg/sensor_data.hpp"
// #include "turtle_control/msg/WheelCommands.msg"
// #include "turtle_control/msg/SensorData.msg"

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
  }

private:
  double wheel_radius, track_width;
  int motor_cmd_max;
  double motor_cmd_per_rad_sec, encoder_ticks_per_rad, collision_radius;
  // more stuff
  // inc timer callback
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleControl>());
  rclcpp::shutdown();
  return 0;
}
