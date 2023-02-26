/// \file
/// \brief implementing ekf slam
///
/// PARAMETERS:
///     rate (double): frequency of timer callback (hertz)
///     x0 (double): initial x position of the robot (m)
///     y0 (double): initial y position of the robot (m)
///     z0 (double): initial z position of the robot (m)
///     theta0 (double): initial heading of the robot (rad)
///     cyl_radius (double): radius of obstacles (m)
///     cyl_height (double): height of obstacles (m)
///     obstacles.x (std::vector<double>): x coordinates of obstacles (m)
///     obstacles.y (std::vector<double>): y coordinates of obstacles (m)
///     x_length (double): length of walls in x-dir (m)
///     y_length (double): length of walls in y-dir (m)
///     input_noise (double): initial heading of the robot (rad)
///     #TODO: add all parameters, update publishers, etc
///      (std::vector<double>): x coordinates of obstacles (m)
///      (std::vector<double>): y coordinates of obstacles (m)
///
/// PUBLISHES:
///     obstacles (visualization_msgs::msg::MarkerArray): publish cylinder markers to rviz
///     timestep (int): publish timestep of simulation
/// SUBSCRIBES:
///     none
/// SERVERS:
///     reset (std_srvs::srv::Empty): reset timestep to 0
///     teleport (nusim::srv::Teleport): move the robot to a specified x, y, theta position
/// CLIENTS:
///     none

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "turtlelib/diff_drive.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class Ekf_slam : public rclcpp::Node
{
  public:
    Ekf_slam():
      Node("slam"),
      rate(5.0),
      timestep(0),
      state_curr(std::vector<double>{0.0,0.0, 0.0}),
      state_next(std::vector<double>{0.0,0.0, 0.0}),
      wheel_radius(0.033),
      track_width(0.16), 
      motor_cmd_per_rad_sec(1.0 / 0.024), 
      encoder_ticks_per_rad(651.8986)
    {
      declare_parameter("wheel_radius", rclcpp::ParameterValue(-1.0));
      declare_parameter("track_width", rclcpp::ParameterValue(-1.0));
      declare_parameter("motor_cmd_per_rad_sec", rclcpp::ParameterValue(-1.0));
      declare_parameter("encoder_ticks_per_rad", rclcpp::ParameterValue(-1.0));
      get_parameter("wheel_radius", wheel_radius);
      get_parameter("track_width", track_width);
      get_parameter("motor_cmd_per_rad_sec", motor_cmd_per_rad_sec);
      get_parameter("encoder_ticks_per_rad", encoder_ticks_per_rad);
      std::vector<double> params_set {wheel_radius, track_width,
                                      motor_cmd_per_rad_sec, encoder_ticks_per_rad};

      for (unsigned int i = 0; i < params_set.size(); i++) {
        if (params_set.at(i) == -1.0) {
          RCLCPP_ERROR(get_logger(), "One or more parameters not set!");
          rclcpp::shutdown();
        }
      }

      wheel_cmd_sub = create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
      "~/wheel_cmd", 10, std::bind(
        &Ekf_slam::wheel_cmd_callback, this,
        std::placeholders::_1));
      sd_sub = create_subscription<nuturtlebot_msgs::msg::SensorData>(
        "~/sensor_data", 10, std::bind(
        &Ekf_slam::sd_callback, this,
        std::placeholders::_1));
      timer = 
        create_wall_timer(
          std::chrono::milliseconds(int(1.0 / rate * 1000)),
          std::bind(&Ekf_slam::timer_callback, this));
    }

  private:
    double rate;
    size_t timestep;
    std::vector<double> state_curr, state_next;
    double wheel_radius, track_width, motor_cmd_per_rad_sec, encoder_ticks_per_rad;
    rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_sub;
    rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sd_sub;
    rclcpp::TimerBase::SharedPtr timer;
    
    turtlelib::Twist2D u{0.0, 0.0, 0.0};


    /// \brief main timer callback
    void timer_callback() {
      ; // do stuff
    }

    /// \brief Wheel_cmd subscription
    /// \param cmd - wheel command message
    void wheel_cmd_callback(nuturtlebot_msgs::msg::WheelCommands)
    {
      // std::vector<double> wheel_velocities{0.0, 0.0};
      // wheel_velocities.at(0) = cmd.left_velocity / motor_cmd_per_rad_sec; // * max_rot_vel / 265.0;
      // wheel_velocities.at(1) = cmd.right_velocity / motor_cmd_per_rad_sec; // * max_rot_vel / 265.0;
      // if (wheel_velocities.at(0) != 0.0 && wheel_velocities.at(1) != 0.0) {
      //   wheel_velocities.at(0) += gauss_dist_vel_noise(get_random());
      //   wheel_velocities.at(1) += gauss_dist_vel_noise(get_random());
      // }
    }

    /// \brief sensor data callback
    /// \param sd - sensor data input
    void sd_callback(nuturtlebot_msgs::msg::SensorData sd) {
      encoder_to_rad(sd.left_encoder, sd.right_encoder);
    }

    /// \brief helper function to convert encoder data to radians  
    /// \param left_encoder - encoder ticks, left wheel  
    /// \param right_encoder - encoder ticks, right wheel
    /// \return radian values for each wheel 
    std::vector<double> encoder_to_rad(int32_t left_encoder, int32_t right_encoder) {
      return std::vector<double> {double(left_encoder)/encoder_ticks_per_rad, double(right_encoder)/encoder_ticks_per_rad};
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Ekf_slam>());
  rclcpp::shutdown();
  return 0;
}
