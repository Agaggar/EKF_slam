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
    vec qt, qt_minusone, dq, mt_minusone, mt, zeta_minusone, zeta, rj, phij, m_seen, zjt, zhat_jt;
    std::vector<double> wheel_rad;
    mat bigAt = mat(3, 3);
    mat sys_cov_minusone = mat(3, 3, arma::fill::zeros);
    mat Q, Qbar, sys_cov, Hj;
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
    double dxj, dyj, dj;
    
    turtlelib::Twist2D u{0.0, 0.0, 0.0};


    /// \brief main timer callback
    void timer_callback() {
      if (timestep == 0) {
        qt_minusone = {greenbot.getCurrentConfig().at(2), greenbot.getCurrentConfig().at(0), greenbot.getCurrentConfig().at(1)};
        // RCLCPP_INFO(get_logger(), "state: %f, %f, %f", qt_minusone.at(0), qt_minusone.at(1), qt_minusone.at(2));
        // qt_minusone = qt;
        if (wheel_rad.size() < 2) {
          // RCLCPP_INFO(get_logger(), "Wheel encoder never set. Defaulting...");
          wheel_rad = {0.0, 0.0};
        }
        if (bigAt.n_elem < 9) {
          // RCLCPP_INFO(get_logger(), "mt_size: %ld", mt_minusone.at(0).size());
          // bigAt.set_size(3, 3);
          bigAt.zeros(size(3,3));
        }
        Q = arma::eye(3, 3);
        Hj.zeros(2, 3);
      }
      else {
        qt_minusone = {greenbot.getCurrentConfig().at(2), greenbot.getCurrentConfig().at(0), greenbot.getCurrentConfig().at(1)};
        greenbot.fkinematics(std::vector<double>{wheel_rad.at(0) - greenbot.getWheelPos().at(0), wheel_rad.at(1) - greenbot.getWheelPos().at(1)});
        dq = {greenbot.getCurrentConfig().at(2) - qt_minusone.at(0), greenbot.getCurrentConfig().at(0) - qt_minusone.at(1), greenbot.getCurrentConfig().at(1) - qt_minusone.at(2)};
        qt = {greenbot.getCurrentConfig().at(2), greenbot.getCurrentConfig().at(0), greenbot.getCurrentConfig().at(1)};
        dq = {turtlelib::normalize_angle(qt(0) - qt_minusone(0)), qt(1) - qt_minusone(1), qt(2) - qt_minusone(2)};
        zeta_minusone = arma::join_cols(qt_minusone, mt_minusone);
        zeta = arma::join_cols(qt, mt_minusone); // since mt = mt_minusone in our prediction (the landmarks don't move)
        create_At();
        create_cov();
        measurement_model();
        // do stuff to UPDATE qt
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
      js_msg.position = wheel_rad;
      js_msg.velocity = std::vector<double>{wheel_rad.at(0) - greenbot.getWheelPos().at(0), wheel_rad.at(1) - greenbot.getWheelPos().at(1)};
      js_pub->publish(js_msg);

      timestep += 1;
    }

    /// \brief sensor data callback
    /// \param sd - sensor data input
    void sd_callback(nuturtlebot_msgs::msg::SensorData sd) {
      wheel_rad = encoder_to_rad(sd.left_encoder, sd.right_encoder);
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
      if (mt_minusone.n_elem != 2*count) {
        RCLCPP_INFO(get_logger(), "here 2.1.1");
        // mt_minusone.set_size(2*count);
        mt_minusone.zeros(2*count);
      }
      if (rj.n_elem != count) {
        // rj.set_size(count);
        rj.zeros(2*count);
      }
      if (phij.n_elem != count) {
        // phij.set_size(count);
        phij.zeros(2*count);
      }
      if (m_seen.n_elem != count) {
        // m_seen.set_size(count);
        m_seen.zeros(count);
      }
      count = 0;
      for (size_t loop=0; loop < obs.markers.size(); loop++) {
        if (obs.markers.at(loop).action != 2) {
          mt_minusone(2*count) = (obs.markers.at(loop).pose.position.x);
          mt_minusone(2*count + 1) = (obs.markers.at(loop).pose.position.y);
          // if seen, else
          if (turtlelib::almost_equal(rj(count), distance(0.0, 0.0, mt_minusone(2*count), mt_minusone(2*count + 1)), 0.01) &&
              turtlelib::almost_equal(phij(count), turtlelib::normalize_angle(atan2(mt_minusone(2*count + 1), mt_minusone(2*count))))) {
                m_seen(count) = 1;
          }
          else {
            rj(count) = distance(0.0, 0.0, mt_minusone(2*count), mt_minusone(2*count + 1));
            phij(count) = turtlelib::normalize_angle(atan2(mt_minusone(2*count + 1), mt_minusone(2*count)));
            m_seen(count) = 0;
          }
          count++;
        }
      }
      zjt = arma::join_cols(rj, phij);
      zhat_jt = arma::join_cols(rj, phij); // this will be later updated, but for now it's the same size as zjt
      if (bigAt.n_elem != (3+2*count)*(3+2*count)) {
        // bigAt.set_size(3+2*count, 3+2*count);
        bigAt.zeros(3+2*count, 3+2*count);
        RCLCPP_INFO(get_logger(), "bigAt resize: %lld", bigAt.n_rows);
      }

    }

    /// \brief helper function to create framework for bigAt
    void create_At() {
      bigAt.eye();
      bigAt.at(1, 1) = -dq.at(2);
      bigAt.at(2, 1) = dq.at(1);
      // create Qbar, if new landmarks are found (including at t=0)
      if (Qbar.n_elem != bigAt.n_elem) {
        Qbar = mat(bigAt.n_rows, bigAt.n_cols, arma::fill::zeros);
        for (int i = 0; i < 3; i++) {
          for (int j = 0; i < 3; i++) {
            Qbar(i, j) = Q(i, j);
          }
        }
      }
    }

    /// \brief helper function to create system covariance matrix 
    void create_cov() {
      if (sys_cov_minusone.n_elem != bigAt.n_elem) {
        // initialize new landmarks 
        size_t old_n = sys_cov_minusone.n_rows;
        sys_cov_minusone.zeros(3+mt_minusone.n_elem, 3+mt_minusone.n_elem);
        for (size_t n = old_n; n < mt_minusone.n_elem/2; n++) {
          sys_cov_minusone(3+n,3+n) = 100000; // large value for unknown diagonal measurements
        }
        sys_cov = sys_cov_minusone;
        RCLCPP_INFO(get_logger(), "here 6");
      }
      for (size_t i = 0; i < bigAt.n_rows; i++) {
        for (size_t j = 0; j < bigAt.n_cols; j++) {
          RCLCPP_INFO(get_logger(), "diag A, %f, cov: %f, Qbar: %f", bigAt(i, j), sys_cov_minusone(i, j), Qbar(i, j));
        }
      }
      // arma::mat mul not working
      // RCLCPP_INFO(get_logger(), "sizes: %lld, %lld, %lld, %lld", bigAt.n_rows, sys_cov_minusone.n_rows, bigAt.n_cols, sys_cov_minusone.n_cols);
      // sys_cov = bigAt * sys_cov_minusone * bigAt.t() + Qbar;
    }

    /// \brief function to create measurement model
    void measurement_model() {
      if (Hj.n_elem != (2*(3+2*m_seen.n_elem))) {
        Hj.zeros(2, (3+2*m_seen.n_elem));
      }
      for (size_t j = 0; j < m_seen.n_elem; j++) {
        if (!m_seen(j)) {
          zeta(3+2*j) = zeta(1) + rj(j)*cos(phij(j) + zeta(0)); // update measured landmark x
          zeta(3+2*j+1) = zeta(2) + rj(j)*sin(phij(j) + zeta(0)); // update measured landmark y
        }
        dxj =  zeta(3+2*j) - zeta(1);
        dyj = zeta(3+2*j+1) - zeta(2);
        dj = dxj*dxj + dyj*dyj;
        zhat_jt(j) =  sqrt(dj);
        zhat_jt(j + m_seen.n_elem) = turtlelib::normalize_angle(atan2(dyj, dxj) - zeta(0)); 

      }

    }

    /// \brief helper function to convert encoder data to radians  
    /// \param left_encoder - encoder ticks, left wheel  
    /// \param right_encoder - encoder ticks, right wheel
    /// \return radian values for each wheel 
    std::vector<double> encoder_to_rad(int32_t left_encoder, int32_t right_encoder) {
      return std::vector<double> {double(left_encoder)/encoder_ticks_per_rad, double(right_encoder)/encoder_ticks_per_rad};
    }

    /// \brief helper function to check distance between points
    /// \param origin_x - current x coordinate
    /// \param origin_y - current y coordinate
    /// \param point_x - target x coordinate
    /// \param point_y - target y coordinate
    /// \return distance
    double distance(double origin_x, double origin_y, double point_x, double point_y) {
      return sqrt(pow((origin_x - point_x), 2.0) + pow((origin_y - point_y), 2.0));
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Ekf_slam>());
  rclcpp::shutdown();
  return 0;
}
