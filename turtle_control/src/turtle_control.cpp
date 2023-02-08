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
///     none
/// CLIENTS:
///     none

#include <cstdint>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "turtlelib/diff_drive.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

class TurtleControl : public rclcpp::Node
{
public:
  TurtleControl()
  : Node("turtle_control"),
    wheel_radius(0.033),
    track_width(0.16), 
    motor_cmd_max(265),
    motor_cmd_per_rad_sec(1.0 / 0.024), 
    encoder_ticks_per_rad(651.8986),
    collision_radius(0.11),
    nubot()
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

    nubot.setWheelRadius(wheel_radius);
    nubot.setWheelTrack(track_width);

    wheel_cmd_pub = create_publisher<nuturtlebot_msgs::msg::WheelCommands>("/wheel_cmd", 10);
    js_pub = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    cmd_vel_sub = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&TurtleControl::cmd_vel_callback, this, std::placeholders::_1));
    sensor_data_sub = create_subscription<nuturtlebot_msgs::msg::SensorData>("/sensor_data", 10, std::bind(&TurtleControl::sd_callback, this, std::placeholders::_1));
    timer =
      create_wall_timer(
      std::chrono::milliseconds(int(1.0 / 200.0 * 1000)),
      std::bind(&TurtleControl::timer_callback, this)); // timer at 200 Hz
  }

private:
  double wheel_radius, track_width;
  int motor_cmd_max;
  double motor_cmd_per_rad_sec, encoder_ticks_per_rad, collision_radius;
  turtlelib::DiffDrive nubot = turtlelib::DiffDrive(0.0, 0.0, 0.0, 0.0, 0.0);
  rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_pub;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
  rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_sub;
  rclcpp::TimerBase::SharedPtr timer;
  turtlelib::Twist2D qdot{0.0, 0.0, 0.0};
  sensor_msgs::msg::JointState js_msg;
  rclcpp::Time current_time{get_clock()->now()};

  /// \brief Timer callback
  void timer_callback()
  {
    current_time = get_clock()->now();
    // RCLCPP_INFO(get_logger(), "vel to ticks: %f, %f", nubot.ikinematics(qdot).at(0), nubot.ikinematics(qdot).at(1));
    wheel_cmd_pub->publish(conv_vel_to_tick(nubot.ikinematics(qdot)));
    js_pub->publish(js_msg);
  }

  /// \brief cmd_vel subscription callback assigns 2D velocity values to Twist2D qdot
  void cmd_vel_callback(const geometry_msgs::msg::Twist &twist) {
    // RCLCPP_INFO(get_logger(), "updating qdot twist, %f, %f, %f", twist.angular.z, twist.linear.x, twist.linear.y);
    qdot.angular = twist.angular.z;
    qdot.linearx = twist.linear.x;
    qdot.lineary = twist.linear.y;
  }

  /// \brief sensor_data subscription callback
  void sd_callback(const nuturtlebot_msgs::msg::SensorData sd) {
    js_msg.header.stamp = get_clock()->now();
    js_msg.header.frame_id = "blue/base_footprint";
    js_msg.name = std::vector<std::string>{"wheel_left_joint", "wheel_right_joint"};
    js_msg.position = encoder_to_rad(sd.left_encoder, sd.right_encoder);
    js_msg.velocity = compute_vel(js_msg.position, nubot.getWheelPos());
    nubot.fkinematics(js_msg.position);
    // nubot.fkinematics(std::vector<double>{js_msg.position.at(0) - nubot.getWheelPos().at(0),
    //                                       js_msg.position.at(1) - nubot.getWheelPos().at(1)});
    // RCLCPP_INFO(get_logger(), "bluebot pos: %f, %f, %f", nubot.getCurrentConfig().at(0), nubot.getCurrentConfig().at(1), nubot.getCurrentConfig().at(2));
    nubot.setWheelPos(js_msg.position);
  }

  nuturtlebot_msgs::msg::WheelCommands conv_vel_to_tick(std::vector<double> wheel_vel) {
    nuturtlebot_msgs::msg::WheelCommands cmd;
    if ((wheel_vel.at(0) * motor_cmd_per_rad_sec) > motor_cmd_max) {
      cmd.left_velocity = int32_t (motor_cmd_max);  
    }
    else {
      cmd.left_velocity = int32_t (std::round(wheel_vel.at(0) * motor_cmd_per_rad_sec));
    }
    if ((wheel_vel.at(1) * motor_cmd_per_rad_sec) > motor_cmd_max) {
      cmd.right_velocity = int32_t (motor_cmd_max);  
    }
    else {
      cmd.right_velocity = int32_t (std::round(wheel_vel.at(1) * motor_cmd_per_rad_sec));
    }
    // cmd.right_velocity = int32_t (std::fmod(wheel_vel.at(1) * motor_cmd_per_rad_sec, motor_cmd_max));
    return cmd;
  }

  std::vector<double> encoder_to_rad(int32_t left_encoder, int32_t right_encoder) {
    return std::vector<double> {double(left_encoder)/encoder_ticks_per_rad, double(right_encoder)/encoder_ticks_per_rad};
  }

  std::vector<double> compute_vel(std::vector<double> current, std::vector<double> prev) {
    // RCLCPP_INFO(get_logger(), "delta t: %f", js_msg.header.stamp.sec + js_msg.header.stamp.nanosec*1e-9 - current_time.nanoseconds()*1e-9);
    return std::vector<double> {(current.at(0) - prev.at(0))/(js_msg.header.stamp.sec + js_msg.header.stamp.nanosec*1e-9 - current_time.nanoseconds()*1e-9), 
                                (current.at(0) - prev.at(0))/(js_msg.header.stamp.sec + js_msg.header.stamp.nanosec*1e-9 - current_time.nanoseconds()*1e-9)
    };
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleControl>());
  rclcpp::shutdown();
  return 0;
}
