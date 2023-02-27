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
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "turtlelib/diff_drive.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <armadillo>

using namespace std::chrono_literals;
using namespace arma;

class Ekf_slam : public rclcpp::Node
{
  public:
    Ekf_slam():
      Node("slam"),
      rate(5.0),
      timestep(0),
      qt(vec(3)),
      qt_minusone(vec(3)),
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
      tf2_rostf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    js_pub = create_publisher<sensor_msgs::msg::JointState>("~/joint_states", 10);
      timer = 
        create_wall_timer(
          std::chrono::milliseconds(int(1.0 / rate * 1000)),
          std::bind(&Ekf_slam::timer_callback, this));
    }

  private:
    double rate;
    size_t timestep, count;
    vec qt, qt_minusone, dq, mt_minusone, mt;
    std::vector<double> d_wheel_rad;
    mat bigAt = mat(2, 3);
    double wheel_radius, track_width, motor_cmd_per_rad_sec, encoder_ticks_per_rad;
    turtlelib::DiffDrive greenbot = turtlelib::DiffDrive(0.0, 0.0, 0.0, 0.0, 0.0);
    // rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_sub;
    rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sd_sub;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr fake_sensor_sub;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf2_rostf_broadcaster;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub;
    rclcpp::TimerBase::SharedPtr timer;
    geometry_msgs::msg::TransformStamped t;
    tf2::Quaternion q;
    sensor_msgs::msg::JointState js_msg;
    
    turtlelib::Twist2D u{0.0, 0.0, 0.0};


    /// \brief main timer callback
    void timer_callback() {
      if (timestep == 0) {
        qt_minusone = {greenbot.getCurrentConfig().at(2), greenbot.getCurrentConfig().at(0), greenbot.getCurrentConfig().at(1)};
        // RCLCPP_INFO(get_logger(), "state: %f, %f, %f", qt_minusone.at(0), qt_minusone.at(1), qt_minusone.at(2));
        std::cout << bigAt.size();
        // qt_minusone = qt;
        if (d_wheel_rad.size() < 2) {
          // RCLCPP_INFO(get_logger(), "Wheel encoder never set. Defaulting...");
          d_wheel_rad = {0.0, 0.0};
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
      t.header.stamp = get_clock()->now();
      t.header.frame_id = "nusim/world";
      t.child_frame_id = "green/base_footprint";
      t.transform.translation.x = greenbot.getCurrentConfig().at(0);
      t.transform.translation.y = greenbot.getCurrentConfig().at(1);
      t.transform.translation.z = 0.0;
      q.setRPY(0, 0, greenbot.getCurrentConfig().at(2));
      t.transform.rotation.x = q.x();
      t.transform.rotation.y = q.y();
      t.transform.rotation.z = q.z();
      t.transform.rotation.w = q.w();
      tf2_rostf_broadcaster->sendTransform(t);

      js_msg.header.stamp = get_clock()->now();
      js_msg.header.frame_id = "green/base_footprint";
      js_msg.name = std::vector<std::string>{"wheel_left_joint", "wheel_right_joint"};
      js_msg.position = d_wheel_rad; // d_wheel_rad is supposed to be a CHANGE in rad
      js_msg.velocity = d_wheel_rad; // this is supposed to be velocity
      js_pub->publish(js_msg);

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
      count = 0;
      for (size_t loop=0; loop < obs.markers.size(); loop++) {
        if (obs.markers.at(loop).action != 2) {
          count++;
        }
      }
      if (mt_minusone.empty()) {
        mt_minusone.resize(2*count);
      }
      for (size_t loop=0; loop < obs.markers.size(); loop++) {
        if (obs.markers.at(loop).action != 2) {
          mt_minusone(2*loop) = (obs.markers.at(loop).pose.position.x);
          mt_minusone(2*loop + 1) = (obs.markers.at(loop).pose.position.y);
        }
      }
      if (bigAt.size() < (3+2*count)) {
        bigAt.resize(3+2*count, 3+2*count);
        RCLCPP_INFO(get_logger(), "bigAt resize: %lld", bigAt.size());
      }
    }

    /// \brief helper function to create framework for bigAt
    void create_At() {
      bigAt.eye();
      bigAt.at(1, 1) = -dq.at(2);
      bigAt.at(2, 1) = dq.at(1);
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
