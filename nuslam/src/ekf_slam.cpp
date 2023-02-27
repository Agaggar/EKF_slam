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
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

class Ekf_slam : public rclcpp::Node
{
  public:
    Ekf_slam():
      Node("slam"),
      rate(5.0),
      timestep(0),
      qt(std::vector<double>{0.0,0.0, 0.0}),
      qt_minusone(std::vector<double>{0.0,0.0, 0.0}),
      wheel_radius(0.033),
      track_width(0.16), 
      motor_cmd_per_rad_sec(1.0 / 0.024), 
      encoder_ticks_per_rad(651.8986),
      greenbot()
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

      greenbot.setWheelRadius(wheel_radius);
      greenbot.setWheelTrack(track_width);

      // wheel_cmd_sub = create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
      // "~/wheel_cmd", 10, std::bind(
      //   &Ekf_slam::wheel_cmd_callback, this,
      //   std::placeholders::_1));
      sd_sub = create_subscription<nuturtlebot_msgs::msg::SensorData>(
        "~/sensor_data", 10, std::bind(
        &Ekf_slam::sd_callback, this,
        std::placeholders::_1));
      fake_sensor_sub = create_subscription<visualization_msgs::msg::MarkerArray>(
        "~/fake_sensor", 10, std::bind(&Ekf_slam::fake_sensor_callback, this, std::placeholders::_1)
      );
      timer = 
        create_wall_timer(
          std::chrono::milliseconds(int(1.0 / rate * 1000)),
          std::bind(&Ekf_slam::timer_callback, this));
    }

  private:
    double rate;
    size_t timestep, count;
    std::vector<double> qt, qt_minusone, dq, d_wheel_rad;
    std::vector<std::vector<double>> mt_minusone, mt, bigAt;
    double wheel_radius, track_width, motor_cmd_per_rad_sec, encoder_ticks_per_rad;
    turtlelib::DiffDrive greenbot = turtlelib::DiffDrive(0.0, 0.0, 0.0, 0.0, 0.0);
    // rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_sub;
    rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sd_sub;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr fake_sensor_sub;
    rclcpp::TimerBase::SharedPtr timer;
    
    turtlelib::Twist2D u{0.0, 0.0, 0.0};


    /// \brief main timer callback
    void timer_callback() {
      if (timestep == 0) {
        qt_minusone = {greenbot.getCurrentConfig().at(2), greenbot.getCurrentConfig().at(0), greenbot.getCurrentConfig().at(1)};
        RCLCPP_INFO(get_logger(), "state: %f, %f, %f", qt_minusone.at(0), qt_minusone.at(1), qt_minusone.at(2));
        // qt_minusone = qt;
        if (d_wheel_rad.size() < 2) {
          RCLCPP_INFO(get_logger(), "Wheel encoder never set. Defaulting...");
          d_wheel_rad = {5.0, 5.0};
        }
        if (bigAt.size() < 3) {
          // RCLCPP_INFO(get_logger(), "mt_size: %ld", mt_minusone.at(0).size());
          bigAt.resize(3);
        }
      }
      else {
        qt_minusone = {greenbot.getCurrentConfig().at(2), greenbot.getCurrentConfig().at(0), greenbot.getCurrentConfig().at(1)};
        greenbot.fkinematics(std::vector<double>{d_wheel_rad.at(0) - greenbot.getWheelPos().at(0), d_wheel_rad.at(1) - greenbot.getWheelPos().at(1)});
        dq = {greenbot.getCurrentConfig().at(2) - qt_minusone.at(0), greenbot.getCurrentConfig().at(0) - qt_minusone.at(1), greenbot.getCurrentConfig().at(1) - qt_minusone.at(2)};
        qt = {greenbot.getCurrentConfig().at(2), greenbot.getCurrentConfig().at(0), greenbot.getCurrentConfig().at(1)};
        dq = {qt.at(0) - qt_minusone.at(0), qt.at(1) - qt_minusone.at(1), qt.at(2) - qt_minusone.at(2)};
        create_At();
        // do stuff to get qt
      }
      timestep += 1;
    }

    /// \brief sensor data callback
    /// \param sd - sensor data input
    void sd_callback(nuturtlebot_msgs::msg::SensorData sd) {
      d_wheel_rad = encoder_to_rad(sd.left_encoder, sd.right_encoder);
      // in here, convert change in encoder to a twist somehow
    }

    /// \brief obstacle marker callback
    /// \param obs - obstacle array as published in nusim rviz
    void fake_sensor_callback(visualization_msgs::msg::MarkerArray obs) {
      if (mt_minusone.empty()) {
        mt_minusone.resize(2);
      }
      count = 0;
      for (size_t loop=0; loop < obs.markers.size(); loop++) {
        if (obs.markers.at(loop).action != 2) {
          RCLCPP_INFO(get_logger(), "marekr loop");
          mt_minusone.at(0).push_back(obs.markers.at(loop).pose.position.x);
          mt_minusone.at(1).push_back(obs.markers.at(loop).pose.position.y);
          count++;
        }
      }
      if (bigAt.size() < (3+2*count)) {
        bigAt.resize(3+2*count);
        RCLCPP_INFO(get_logger(), "bigAt resize: %ld", bigAt.size());
      }
    }

    /// \brief helper function to create framework for bigAt
    void create_At() {
      RCLCPP_INFO(get_logger(), "bigAt");
      bigAt.at(0) = (std::vector<double>{0, 0, 0, 0});
      RCLCPP_INFO(get_logger(), "bigAt set");
      bigAt.at(1) = (std::vector<double>{-dq.at(2), 0, 0, 0});
      bigAt.at(2) = (std::vector<double>{dq.at(1), 0, 0, 0});
      for (size_t loop=0; loop < mt_minusone.size(); loop++) {
        RCLCPP_INFO(get_logger(), "in da loop");
        bigAt.at(2 + loop) = std::vector<double>{0, 0, 0, 0};
      }
      for (size_t loop=0; loop < mt_minusone.at(0).size(); loop++) {
        bigAt.at(loop).push_back(0);
        bigAt.at(loop).push_back(0);
      }
      add_identity();
    }

    /// \brief helper function to add identity
    void add_identity() {
      for (size_t loop=0; loop < bigAt.at(0).size(); loop++) {
        bigAt.at(loop).at(loop) += 1;
      }
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
