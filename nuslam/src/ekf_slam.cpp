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

using namespace std::chrono_literals;

class Ekf_slam : public rclcpp::Node
{
  public:
    Ekf_slam():
      Node("slam"),
      rate(5),
      state(std::vector<double>{0.0,0.0, 0.0})
    {
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
    std::vector<double> state;
    rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_sub;
    rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sd_sub;
    rclcpp::TimerBase::SharedPtr timer;

    /// \brief main timer callback
    void timer_callback() {
      ; // do stuff
    }

    /// \brief Wheel_cmd subscription
    void wheel_cmd_callback(nuturtlebot_msgs::msg::WheelCommands cmd)
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
    void sd_callback(nuturtlebot_msgs::msg::SensorData) {
      ; // do stuff
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Ekf_slam>());
  rclcpp::shutdown();
  return 0;
}
