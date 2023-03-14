/// \file
/// \brief implementing ekf slam
///
/// PARAMETERS:
///     rate (double): frequency of timer callback (hertz), defaults to 200
///     timestep (size_t): iterations of timer callback
///     qt (arma::vec): current state of the robot, 3x1 vector
///     qt_minusone (arma::vec): prev state of the robot, 3x1 vector
///     wheel_radius (double): radius of the robot's wheels (m)
///     wheel_track (double): radius of distance to wheels (m)
///     cyl_radius (double): radius of obstacles (m)
///     cyl_height (double): height of obstacles (m)
///     q_coeff (double): scalar value to scale Q matrix - process noise
///     r_coeff (double): scalar value to scale R matrix - sensor matrix
///     greenbot (turtlelib::DiffDrive): diff drive object of green robot
/// PUBLISHES:
///     /tf (tf2_ros::TransformBroadcaster): publish transform from map to odom and odom to green/base_footprint
///     green/joint_states (sensor_msgs::msg::JointState): publish joint state of wheels for motion
///     ~/odom (nav_msgs::msg::Odometry): publish odometry
///     ~/map_obstacles (visualization_msgs::msg::MarkerArray): publish map cylinder markers to rviz
///     green/greenpath (nav_msgs::msg::Path): publish points of where the robot has been
/// SUBSCRIBES:
///     ~/joint_states (sensor_msgs::msg::JointState): odometry joint states for forward kinematics
///     ~/fake_sensor (visualization_msgs::msg::MarkerArray): fake_sensor reading from robot
/// SERVERS:
///     none
/// CLIENTS:
///     none

#include <chrono>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "turtlelib/diff_drive.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <armadillo>

using namespace std::chrono_literals;
using namespace arma;

class Ekf_slam : public rclcpp::Node
{
  public:
    Ekf_slam():
      Node("slam"),
      rate(200.0),
      timestep(0),
      min_num_associate(3),
      qt(vec(3)),
      qt_minusone(vec(3)),
      wheel_radius(0.033),
      track_width(0.16), 
      cyl_radius(0.038),
      cyl_height(0.25),
      q_coeff(0.1),
      r_coeff(1.0),
      max_range(1.0),
      greenbot(),
      data_associate(false)
    {
      declare_parameter("wheel_radius", rclcpp::ParameterValue(wheel_radius));
      declare_parameter("track_width", rclcpp::ParameterValue(track_width));
      get_parameter("wheel_radius", wheel_radius);
      get_parameter("track_width", track_width);

      declare_parameter("obstacles.r", rclcpp::ParameterValue(cyl_radius));
      get_parameter("obstacles.r", cyl_radius);
      declare_parameter("obstacles.h", rclcpp::ParameterValue(cyl_height));
      get_parameter("obstacles.h", cyl_height);
      declare_parameter("slam.q_coeff", rclcpp::ParameterValue(q_coeff));
      get_parameter("slam.q_coeff", q_coeff);
      declare_parameter("slam.r_coeff", rclcpp::ParameterValue(r_coeff));
      get_parameter("slam.r_coeff", r_coeff);
      declare_parameter("max_range", rclcpp::ParameterValue(r_coeff));
      get_parameter("max_range", r_coeff);

      declare_parameter("data_associate", rclcpp::ParameterValue(data_associate));
      get_parameter("data_associate", data_associate);
      greenbot.setWheelRadius(wheel_radius);
      greenbot.setWheelTrack(track_width);

      js_sub = create_subscription<sensor_msgs::msg::JointState>(
        "~/joint_states", 100, std::bind(
        &Ekf_slam::js_callback, this,
        std::placeholders::_1));
      fake_sensor_sub = create_subscription<visualization_msgs::msg::MarkerArray>(
        "~/fake_sensor", 100, std::bind(&Ekf_slam::fake_sensor_callback, this, std::placeholders::_1)
      );
      tf2_rostf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
      js_pub = create_publisher<sensor_msgs::msg::JointState>("green/joint_states", 100);
      odom_pub = create_publisher<nav_msgs::msg::Odometry>("~/odom", 100);
      map_obs_pub = create_publisher<visualization_msgs::msg::MarkerArray>("~/map_obstacles", 5);
      green_path_pub = create_publisher<nav_msgs::msg::Path>("green/greenpath", 100);
      timer = 
        create_wall_timer(
          std::chrono::milliseconds(int(1.0 / rate * 1000)),
          std::bind(&Ekf_slam::timer_callback, this));
    }

  private:
    double rate;
    size_t timestep, min_num_associate;
    vec qt, qt_minusone, dq, mt_minusone, mt, zeta_update, zeta_predict, m_seen, zjt, zhat_jt, dz, landmark_temp, num_associated_landmark;
    std::vector<double> wheel_rad;
    mat bigAt = mat(3, 3, arma::fill::zeros);
    mat Q, Qbar, sys_cov_bar, sys_cov, sys_cov_minusone, Hj, Kj, R, Rj, cov_k;
    double wheel_radius, track_width, cyl_radius, cyl_height, q_coeff, r_coeff, rj, phij, max_range;
    turtlelib::DiffDrive greenbot = turtlelib::DiffDrive(0.0, 0.0, 0.0, 0.0, 0.0);
    bool data_associate;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr map_obs_pub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr fake_sensor_sub;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf2_rostf_broadcaster;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub;
    rclcpp::TimerBase::SharedPtr timer;
    geometry_msgs::msg::TransformStamped t_odom_green, t_mo;
    turtlelib::Transform2D T_mr, T_or, T_mo;
    tf2::Quaternion q;
    sensor_msgs::msg::JointState js_green;
    nav_msgs::msg::Odometry odom_msg;
    double dxj, dyj, dj, prev_theta, dk, dstar;
    size_t poss_obs = 5; // container for total possible number of obstacles 
    size_t bigN, landmark_actual, landmark_maha; // for data association
    visualization_msgs::msg::Marker marker;
    visualization_msgs::msg::MarkerArray all_cyl;
    turtlelib::Twist2D u{0.0, 0.0, 0.0};
    nav_msgs::msg::Path green_path;
    geometry_msgs::msg::PoseStamped current_point;
    geometry_msgs::msg::Quaternion q_geom;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr green_path_pub;

    /// \brief main timer callback
    void timer_callback() {
      if (timestep == 0) {
        m_seen.ones(poss_obs);
        m_seen = -10*m_seen;
        create_all_cylinders();
        qt.zeros(3);
        qt_minusone.zeros(3);
        bigAt.zeros(3+2*poss_obs, 3+2*poss_obs);
        qt = {greenbot.getCurrentConfig().at(2), greenbot.getCurrentConfig().at(0), greenbot.getCurrentConfig().at(1)};
        qt_minusone = {greenbot.getCurrentConfig().at(2), greenbot.getCurrentConfig().at(0), greenbot.getCurrentConfig().at(1)};
        mt.zeros(2*poss_obs);
        sys_cov_minusone.zeros(3+2*poss_obs, 3+2*poss_obs);
        bigN = 0;
        num_associated_landmark.zeros(poss_obs);
        for (size_t n = 0; n < poss_obs; n++) {
          sys_cov_minusone(3+2*n,3+2*n) = 1e9; // large value for unknown diagonal measurements
          sys_cov_minusone(3+2*n+1,3+2*n+1) = 1e9; // large value for unknown diagonal measurements
        }
        sys_cov_bar = sys_cov_minusone;
        sys_cov = sys_cov_minusone;
        mt_minusone.zeros(2*poss_obs);
        Q = q_coeff * arma::eye(3, 3);
        R = r_coeff * arma::eye(2*poss_obs, 2*poss_obs);
        Rj = arma::eye(2, 2);
        Qbar.zeros(3+2*poss_obs, 3+2*poss_obs);
        for (int n = 0; n < 3; n++) {
          Qbar(n,n) = Q(n,n);
        }
        sys_cov_bar = sys_cov_minusone;
        Hj.zeros(2, 3+2*poss_obs);
        zeta_predict = arma::join_cols(qt, arma::vec(2*poss_obs, arma::fill::zeros));
        zeta_update = zeta_predict;
        dz.zeros(2);

        t_mo.header.frame_id = "map";
        t_mo.child_frame_id = "green/odom";
        t_mo.transform.translation.z = 0.0;
        
        t_odom_green.header.frame_id = "green/odom";
        t_odom_green.child_frame_id = "green/base_footprint";
        t_odom_green.transform.translation.z = 0.0;
        green_path.header.frame_id = "nusim/world";
        current_point.header.frame_id = "nusim/world";
        current_point.pose.position.z = 0.0;

        if (wheel_rad.size() < 2) {
          wheel_rad = {0.0, 0.0};
        }
      }

      t_odom_green.header.stamp = get_clock()->now();
      t_odom_green.transform.translation.x = greenbot.getCurrentConfig().at(0);
      t_odom_green.transform.translation.y = greenbot.getCurrentConfig().at(1);
      prev_theta = greenbot.getCurrentConfig().at(2);
      q.setRPY(0, 0, greenbot.getCurrentConfig().at(2));
      t_odom_green.transform.rotation.x = q.x();
      t_odom_green.transform.rotation.y = q.y();
      t_odom_green.transform.rotation.z = q.z();
      t_odom_green.transform.rotation.w = q.w();
      // q_geom = tf2::toMsg(q);
      // t_odom_green.transform.rotation = q_geom;

      if (timestep % 50 == 0) {
        green_path.header.stamp = get_clock()->now();
        current_point.header.stamp = get_clock()->now();
        current_point.pose.position.x = greenbot.getCurrentConfig().at(0);
        current_point.pose.position.y = greenbot.getCurrentConfig().at(1);
        green_path.poses.push_back(current_point);
        green_path_pub->publish(green_path);
        if (bigN > 0) {
          dataAssociate();
        }
        if (bigN >= poss_obs) {
          RCLCPP_INFO(get_logger(), "BIG PROBLEM, too many Ns");
        }
        // zeta_predict.save("zeta_" + std::to_string(timestep) + ".txt", arma_ascii);
        // mt_minusone.save("map_" + std::to_string(timestep) + ".txt", arma_ascii);
      }

      tf2_rostf_broadcaster->sendTransform(t_odom_green);
      odom_pub->publish(compute_odom());
      
      T_mr = {turtlelib::Vector2D{zeta_predict(1), zeta_predict(2)}, zeta_predict(0)}; // from slam
      T_or = {turtlelib::Vector2D{t_odom_green.transform.translation.x, t_odom_green.transform.translation.y}, prev_theta}; // from odom
      T_or = T_or.inv();
      T_mo = T_mr*T_or;
      // multiplication is wrong? angle is right
      RCLCPP_ERROR_STREAM(get_logger(), "T_mr \n" << T_mr << "\nT_or\n" << T_or.inv() << "\nT_mo\n" << T_mo);
      // RCLCPP_INFO(get_logger(), "diff in x: %.6f; diff in y: %.6f; diff in theta: %.6f", 
      //             T_mo.translation().x - t_odom_green.transform.translation.x,
      //             T_mo.translation().y - t_odom_green.transform.translation.y,
      //             T_mo.rotation() - prev_theta);
      // RCLCPP_INFO(get_logger(), "green rot: no normal: %.2f; normal: %.2f", T_mo.rotation(), turtlelib::normalize_angle(T_mo.rotation()));
      t_mo.header.stamp = get_clock()->now();
      t_mo.transform.translation.x = T_mo.translation().x;
      t_mo.transform.translation.y = T_mo.translation().y;
      // t_mo.transform.translation.x = 1.0;
      // t_mo.transform.translation.y = 1.0;
      q.setRPY(0, 0, turtlelib::normalize_angle(T_mo.rotation()));
      t_mo.transform.rotation.x = q.x();
      t_mo.transform.rotation.y = q.y();
      t_mo.transform.rotation.z = q.z();
      t_mo.transform.rotation.w = q.w();
      tf2_rostf_broadcaster->sendTransform(t_mo);

      js_green.header.stamp = get_clock()->now();
      js_green.header.frame_id = "green/base_footprint";
      js_green.name = std::vector<std::string>{"wheel_left_joint", "wheel_right_joint"};
      js_green.position = wheel_rad;
      js_green.velocity = std::vector<double>{wheel_rad.at(0) - greenbot.getWheelPos().at(0), wheel_rad.at(1) - greenbot.getWheelPos().at(1)};
      js_pub->publish(js_green);

      odom_pub->publish(compute_odom());

      update_all_cylinders();
      map_obs_pub->publish(all_cyl);

      timestep += 1;
    }

    /// \brief sensor data callback at 200 Hz
    /// \param sd - sensor data input
    void js_callback(sensor_msgs::msg::JointState js) {
      if ((js.position.size() > 1)) {
        wheel_rad = {js.position.at(0), js.position.at(1)};
        greenbot.fkinematics(std::vector<double>{wheel_rad.at(0) - greenbot.getWheelPos().at(0), wheel_rad.at(1) - greenbot.getWheelPos().at(1)});
      }
      qt = {greenbot.getCurrentConfig().at(2), greenbot.getCurrentConfig().at(0), greenbot.getCurrentConfig().at(1)}; // current odom        
    }

    /// \brief obstacle marker callback at 5 Hz
    /// \param obs - obstacle array as published in nusim rviz
    void fake_sensor_callback(visualization_msgs::msg::MarkerArray obs) {
      /// PREDICT step
      dq = {turtlelib::normalize_angle(qt(0) - qt_minusone(0)), qt(1) - qt_minusone(1), qt(2) - qt_minusone(2)};
      create_At();
      // RCLCPP_ERROR_STREAM(get_logger(), "bigAt: \n" << bigAt);
      // zeta_predict = arma::join_cols(qt_minusone + dq, mt_minusone);
      // RCLCPP_ERROR_STREAM(get_logger(), "zeta_predict 0: \n" << zeta_predict);
      zeta_predict(0) += dq(0);
      zeta_predict(0) = turtlelib::normalize_angle(zeta_predict(0));
      zeta_predict(1) += dq(1);
      zeta_predict(2) += dq(2);
      // RCLCPP_ERROR_STREAM(get_logger(), "dq: \n" << dq);
      // zeta_minusone = arma::join_cols(qt_minusone, mt_minusone);
      create_cov();
      // RCLCPP_ERROR_STREAM(get_logger(), "covariance: \n" << sys_cov_bar);
      
      /// CORRECT step 
      for (size_t landmark_id=0; landmark_id < obs.markers.size(); landmark_id++) {
        if (obs.markers.at(landmark_id).action != 2) {
          rj = distance(0.0, 0.0, obs.markers.at(landmark_id).pose.position.x, obs.markers.at(landmark_id).pose.position.y);
          phij = turtlelib::normalize_angle(atan2(obs.markers.at(landmark_id).pose.position.y, obs.markers.at(landmark_id).pose.position.x));
          if (m_seen.at(landmark_id) != obs.markers.at(landmark_id).id) {
            // RCLCPP_INFO(get_logger(), "now you see landmark: %ld", landmark_id);
            m_seen(landmark_id) = obs.markers.at(landmark_id).id;
            zeta_predict(3 + 2*landmark_id) = zeta_predict(1) + rj*cos(phij + zeta_predict(0)); // update measured landmark x
            zeta_predict(3 + 2*landmark_id + 1) = zeta_predict(2) + rj*sin(phij + zeta_predict(0)); // update measured landmark y
            bigN += 1;
          }
          zjt = {rj, phij};
          measurement_model(landmark_id);
          if (data_associate) {
            dataAssociate();
          }
          zeta_update = zeta_predict + Kj*(zjt - zhat_jt);
          zeta_update(0) = turtlelib::normalize_angle(zeta_update(0));
          // RCLCPP_ERROR_STREAM(get_logger(), "zeta_update: \n" << zeta_update);
          zeta_predict = zeta_update;
          // RCLCPP_ERROR_STREAM(get_logger(), "zeta_predict 3: \n" << zeta_predict);
          sys_cov = (mat(3+2*poss_obs, 3+2*poss_obs, arma::fill::eye) - Kj*Hj) * sys_cov_bar;
          sys_cov_bar = sys_cov;
          // RCLCPP_ERROR_STREAM(get_logger(), "updated cov: \n" << sys_cov_bar);
        }
      }

      qt_minusone = {zeta_predict(0), zeta_predict(1), zeta_predict(2)};
    }

    /// \brief helper function to create framework for bigAt
    void create_At() {
      bigAt.eye();
      bigAt.at(1, 0) = -dq.at(2);
      bigAt.at(2, 0) = dq.at(1);
    }

    /// \brief helper function to create system covariance matrix 
    void create_cov() {
      sys_cov_bar = bigAt * sys_cov_minusone * (bigAt.t()) + Qbar;
      sys_cov_minusone = sys_cov_bar;
    }

    /// \brief function to create measurement model
    void measurement_model(size_t landmark_id) {
      dxj =  getLandmarkX(landmark_id) - zeta_predict(1);
      dyj =  getLandmarkY(landmark_id) - zeta_predict(2);
      dj = dxj*dxj + dyj*dyj;
      zhat_jt =  {sqrt(dj), turtlelib::normalize_angle(atan2(dyj, dxj) - zeta_predict(0))};
      // RCLCPP_ERROR_STREAM(get_logger(), "mt_minusone: \n" << mt_minusone);
      // RCLCPP_ERROR_STREAM(get_logger(), "zeta_predict 1: \n" << zeta_predict);
      // RCLCPP_INFO(get_logger(), "dx: %.4f, dy: %.4f, d: %.4f, phi_calc: %.4f, phi_acc: %.4f", dxj, dyj, dj, atan2(dyj, dxj), zeta_predict(0));
      // RCLCPP_ERROR_STREAM(get_logger(), "z, zhat: \n" << zjt << "\n" << zhat_jt);
      // RCLCPP_INFO(get_logger(), "distance: %.4f, angle: %.4f", rj, phij);
      Hj.zeros(2, 3+2*poss_obs);
      populate_Hj(landmark_id);
      Kj.zeros(3+2*poss_obs, 2);
      Rj(0, 0) = R(2*landmark_id, 2*landmark_id);
      Rj(0, 1) = R(2*landmark_id, 2*landmark_id+1);
      Rj(1, 0) = R(2*landmark_id+1, 2*landmark_id);
      Rj(1, 1) = R(2*landmark_id+1, 2*landmark_id+1);
      Kj = sys_cov_bar*(Hj.t())*((Hj*sys_cov_bar*(Hj.t()) + Rj).i());
      // RCLCPP_ERROR_STREAM(get_logger(), "Kj: \n" << Kj);
    }

    /// \brief helper function to create Hj
    /// \param landmark_id - number of landmark seen
    void populate_Hj(size_t landmark_id) {
      // RCLCPP_INFO(get_logger(), "landmark: %ld", landmark_id);
      Hj(0, 1) = -dxj/sqrt(dj);
      Hj(0, 2) = -dyj/sqrt(dj);
      Hj(1, 0) = -1;
      Hj(1, 1) = dyj/dj;
      Hj(1, 2) = -dxj/dj;

      Hj(0, 3+2*landmark_id) = dxj/sqrt(dj);
      Hj(0, 3+2*landmark_id+1) = dyj/sqrt(dj);
      Hj(1, 3+2*landmark_id) = -dyj/dj;
      Hj(1, 3+2*landmark_id+1) = dxj/dj;
    }

    void dataAssociate() {
      // vec landmark_j = {getLandmarkX(j), getLandmarkY(j)};
      landmark_temp = {getLandmarkX(bigN-1), getLandmarkY(bigN-1)}; // temp set to latest observed
      for (size_t measurement = 0; measurement < bigN; measurement++) {
        landmark_maha = bigN;
        dstar = max_range;
        for (size_t landmark_k = 0; landmark_k <= bigN; landmark_k++) {
          measurement_model(landmark_k);
          cov_k = Hj * sys_cov_bar * (Hj.t()) + Rj;
          // RCLCPP_ERROR_STREAM(get_logger(), "cov_k: \n" << cov_k);
          dk = mahalanobis(cov_k, landmark_temp, zhat_jt);
          if (dk < dstar) {
            dstar = dk;
            landmark_maha = landmark_k;
          }
          // RCLCPP_INFO(get_logger(), "%ld, %ld maha: %.4f", measurement, landmark_k, dk);
        }
        if (landmark_maha == measurement) {
          // associate me!
          num_associated_landmark(measurement) += 1;
          // RCLCPP_INFO(get_logger(), "associating %ld with %ld, maha_dist: %.4f for the %.1f time", measurement, landmark_maha, dstar, num_associated_landmark(measurement));
        }
        else if (landmark_maha == bigN) {
          // new landmark!
          bigN++;
        }
        if (num_associated_landmark(measurement) > min_num_associate) {
          // set bigNth obstacle x and y based on the temp
          zeta_predict(3 + 2*bigN) = getLandmarkX(bigN-1);
          zeta_predict(4 + 2*bigN) = getLandmarkY(bigN-1);
          dxj =  getLandmarkX(bigN-1) - zeta_predict(1);
          dyj =  getLandmarkY(bigN-1) - zeta_predict(2);
          dj = dxj*dxj + dyj*dyj;
          // update zhat
          zhat_jt =  {sqrt(dj), turtlelib::normalize_angle(atan2(dyj, dxj) - zeta_predict(0))};
        }
      }
    }

    /// @brief compute mahalanobis distance
    /// @param covK - covariance matrix
    /// @param xi - measurement
    /// @param xk - prediction
    /// @param temp - temporary matrix (identity 2x2 for 2D application (r, theta))
    /// @return mahalanobis distance
    double mahalanobis(mat covK, vec xi, vec xk, mat temp = arma::eye(2, 2)) {
      temp = ((xi - xk).t()) * (covK.i()) * ((xi - xk));
      return temp(0, 0);
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

    /// \brief helper function to return landmark's x
    /// \param landmark_id - landmark number
    /// \return x coordinate of landmark
    double getLandmarkX(size_t landmark_id) {
      return zeta_predict(3 + 2*landmark_id);
    }
  
    /// \brief helper function to return landmark's y
    /// \param landmark_id - landmark number
    /// \return y coordinate of landmark
    double getLandmarkY(size_t landmark_id) {
      return zeta_predict(4 + 2*landmark_id);
    }

  /// \brief helper function to compute and return an odometry message
  /// \return odometry message
  nav_msgs::msg::Odometry compute_odom()
  {
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = get_clock()->now();
    odom_msg.header.frame_id = "green/odom";
    odom_msg.child_frame_id = "green/base_footprint";
    odom_msg.pose.pose.position.x = greenbot.getCurrentConfig().at(0);
    odom_msg.pose.pose.position.y = greenbot.getCurrentConfig().at(1);
    odom_msg.pose.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, greenbot.getCurrentConfig().at(2));
    q.normalize();
    geometry_msgs::msg::Quaternion q_geom = tf2::toMsg(q);
    odom_msg.pose.pose.orientation = q_geom;
    std::array<double, 36> cov;
    cov.fill(0.0);
    odom_msg.pose.covariance = cov;
    turtlelib::Twist2D vb{0.0, 0.0, 0.0};
    if ((js_green.velocity.size() > 0)) {
      vb = greenbot.velToTwist(js_green.velocity);
    }
    odom_msg.twist.twist.linear.x = vb.linearx;
    odom_msg.twist.twist.linear.y = vb.lineary;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = vb.angular;
    odom_msg.twist.covariance = cov;
    return odom_msg;
  }

  /// \brief Create a single cylinder and add it to a MarkerArray
  /// \param loop - landmark number
  void create_cylinder(size_t loop)
  {
    marker.header.frame_id = "map";
    marker.header.stamp = get_clock()->now();
    marker.id = loop;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    if ((m_seen(loop) >= 0.0)) {
      marker.action = visualization_msgs::msg::Marker::ADD;
    }
    else {
      marker.action = visualization_msgs::msg::Marker::DELETE;
    }
    marker.color.a = 1.0;
    marker.color.r = 120.0 / 256.0;
    marker.color.g = 149.0 / 256.0;
    marker.color.b = 131.0 / 256.0;
    marker.scale.x = cyl_radius;
    marker.scale.y = cyl_radius;
    marker.scale.z = cyl_height;
    marker.pose.position.z = cyl_height / 2.0;
    all_cyl.markers.push_back(marker);
  }

  /// \brief Create all cylinders
  void create_all_cylinders()
  {
    for (size_t loop = 0; loop < poss_obs; loop++) {
      create_cylinder(loop);
    }
  }

  /// \brief Update positions, timesteps, etc when publishing
  void update_all_cylinders()
  {
    for (size_t loop = 0; loop < poss_obs; loop++) {
      all_cyl.markers.at(loop).header.stamp = get_clock()->now();
      if (m_seen(loop) >= 0.0) {
        all_cyl.markers.at(loop).action = visualization_msgs::msg::Marker::ADD;
        all_cyl.markers.at(loop).pose.position.x = zeta_predict(qt.n_elem + loop*2);
        all_cyl.markers.at(loop).pose.position.y = zeta_predict(qt.n_elem + loop*2 + 1);
      }
      else {
        all_cyl.markers.at(loop).action = visualization_msgs::msg::Marker::DELETE;
      }
    }
  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Ekf_slam>());
  rclcpp::shutdown();
  return 0;
}
