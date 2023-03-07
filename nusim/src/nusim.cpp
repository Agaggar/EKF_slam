/// \file
/// \brief spin nusim node to simulat one robot and several obstacles
///
/// PARAMETERS:
///     rate (double): frequency of timer callback (hertz)
///     x0 (double): initial x position of the robot (m)
///     y0 (double): initial y position of the robot (m)
///     z0 (double): initial z position of the robot (m)
///     theta0 (double): initial heading of the robot (rad)
///     cyl_radius (double): radius of obstacles (m)
///     cyl_height (double): height of obstacles (m)
///     obs_x (std::vector<double>): x coordinates of obstacles (m)
///     obs_y (std::vector<double>): y coordinates of obstacles (m)
///     x_length (double): length of walls in x-dir (m)
///     y_length (double): length of walls in y-dir (m)
///     input_noise (double): initial heading of the robot (rad)
///     slip_fraction (double): parameter to simulate slipping of wheel (dimensionless)
///     basic_sensor_variance (double): STD_DEV of the sensor reading (dimensionless)
///     max_range (double): maximum sensing range of sensor (m)
///     collision_radius (double): loaded from diff_params.yaml; collision radius in m
///     encoder_ticks_per_rad (double): loaded from diff_params.yaml; number of encoder ticks per radian
///     motor_cmd_per_rad_sec (double): loaded from diff_params.yaml; motor command tick in rad/s
///     draw_only (bool): red robot appears and draw path
///     angle_min (double): minimum angle that lidar can sense (rad)
///     angle_max (double): maximum angle that lidar can sense (rad)
///     angle_increment (double): angle increments between lidar data (rad)
///     time_increment (double): time between each reading of lidar data (s)
///     scan_time (double): total time per full lidar measurement (s)
///     range_min (double): minimum range that lidar can sense (m)
///     range_max (double): maximum range that lidar can sense, according to its datasheet (m)
///     lidar_noise (double): value to simulate noise in lidar measurements (dimensionless)
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
#include <cmath>
#include <random>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "nusim/srv/teleport.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "turtlelib/diff_drive.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/path.hpp"

using namespace std::chrono_literals;

class Nusim : public rclcpp::Node
{
public:
  Nusim()
  : Node("nusim"),
    timestep(0),
    rate(200.0),
    x0(0.0),
    y0(0.0),
    z0(0.0),
    theta0(0.0),
    cyl_radius(0.038),
    cyl_height(0.25),
    obs_x(std::vector<double> {0.0}),
    obs_y(std::vector<double> {0.0}),
    x_length(2.5),
    y_length(2.5),
    input_noise(0.0),
    slip_fraction(0.0),
    basic_sensor_variance(1.0),
    max_range(1.0),
    collision_radius(0.11),
    encoder_ticks_per_rad(651.8986),
    motor_cmd_per_rad_sec(1.0 / 0.024),
    draw_only(true),
    angle_min(0.0),
    angle_max(6.2657318115234375),
    angle_increment(0.01745329238474369),
    time_increment(0.0005574136157520115),
    scan_time(0.20066890120506287),
    range_min(0.11999999731779099),
    range_max(3.5),
    lidar_noise(0.0001)
  {
    rcl_interfaces::msg::ParameterDescriptor rate_param_desc;
    rate_param_desc.name = "rate";
    rate_param_desc.type = 3;         // rate is a double
    rate_param_desc.description = "simulation refresh rate (hz)";
    // rate defaults to 200.0
    declare_parameter("rate", rclcpp::ParameterValue(rate), rate_param_desc);
    get_parameter("rate", rate);

    rcl_interfaces::msg::ParameterDescriptor x0_param_desc;
    x0_param_desc.name = "x0";
    x0_param_desc.type = 3;         // x0 is a double
    x0_param_desc.description = "initial x0 position (m)";
    // x0 defaults to 0.0
    declare_parameter("x0", rclcpp::ParameterValue(x0), x0_param_desc);
    get_parameter("x0", x0);

    rcl_interfaces::msg::ParameterDescriptor y0_param_desc;
    y0_param_desc.name = "y0";
    y0_param_desc.type = 3;
    y0_param_desc.description = "initial y0 position (m)";
    // y0 defaults to 0.0
    declare_parameter("y0", rclcpp::ParameterValue(y0), y0_param_desc);
    get_parameter("y0", y0);

    rcl_interfaces::msg::ParameterDescriptor z0_param_desc;
    z0_param_desc.name = "z0";
    z0_param_desc.type = 3;
    z0_param_desc.description = "initial z0 position (m)";
    // z0 defaults to 0.0
    declare_parameter("z0", rclcpp::ParameterValue(z0), z0_param_desc);
    get_parameter("z0", z0);

    rcl_interfaces::msg::ParameterDescriptor theta0_param_desc;
    theta0_param_desc.name = "theta0";
    theta0_param_desc.type = 3;
    theta0_param_desc.description = "initial theta value (rad)";
    // theta0 defaults to 0.0
    declare_parameter("theta0", rclcpp::ParameterValue(theta0), theta0_param_desc);
    get_parameter("theta0", theta0);

    rcl_interfaces::msg::ParameterDescriptor cyl_radius_param_desc;
    cyl_radius_param_desc.name = "obstacles.r";
    cyl_radius_param_desc.type = 3;
    cyl_radius_param_desc.description = "radius of cylinder obstacles (m)";
    // defaults to 0.038
    declare_parameter("obstacles.r", rclcpp::ParameterValue(cyl_radius), cyl_radius_param_desc);
    get_parameter("obstacles.r", cyl_radius);

    declare_parameter("obstacles.h", rclcpp::ParameterValue(cyl_height));
    get_parameter("obstacles.h", cyl_height);

    rcl_interfaces::msg::ParameterDescriptor obs_x_param_desc;
    obs_x_param_desc.name = "obstacles.x";
    obs_x_param_desc.type = 8; // double array
    obs_x_param_desc.description = "x coordinates of cylinder obstacles (m)";
    // defaults to the array in initializer list
    declare_parameter("obstacles.x", rclcpp::ParameterValue(obs_x), obs_x_param_desc);
    get_parameter("obstacles.x", obs_x);

    rcl_interfaces::msg::ParameterDescriptor obs_y_param_desc;
    obs_x_param_desc.name = "obstacles.y";
    obs_x_param_desc.type = 8; // double array
    obs_x_param_desc.description = "y coordinates of cylinder obstacles (m)";
    // defaults to the array in initializer list
    declare_parameter("obstacles.y", rclcpp::ParameterValue(obs_y), obs_y_param_desc);
    get_parameter("obstacles.y", obs_y);

    declare_parameter("x_length", rclcpp::ParameterValue(x_length));
    get_parameter("x_length", x_length);

    declare_parameter("y_length", rclcpp::ParameterValue(y_length));
    get_parameter("y_length", y_length);

    declare_parameter("draw_only", rclcpp::ParameterValue(draw_only));
    get_parameter("draw_only", draw_only);

    declare_parameter("input_noise", rclcpp::ParameterValue(input_noise));
    get_parameter("input_noise", input_noise);

    declare_parameter("slip_fraction", rclcpp::ParameterValue(slip_fraction));
    get_parameter("slip_fraction", slip_fraction);

    declare_parameter("basic_sensor_variance", rclcpp::ParameterValue(basic_sensor_variance));
    get_parameter("basic_sensor_variance", basic_sensor_variance);

    declare_parameter("collision_radius", rclcpp::ParameterValue(collision_radius));
    get_parameter("collision_radius", collision_radius);

    declare_parameter("encoder_ticks_per_rad", rclcpp::ParameterValue(encoder_ticks_per_rad));
    get_parameter("encoder_ticks_per_rad", encoder_ticks_per_rad);

    declare_parameter("motor_cmd_per_rad_sec", rclcpp::ParameterValue(motor_cmd_per_rad_sec));
    get_parameter("motor_cmd_per_rad_sec", motor_cmd_per_rad_sec);

    declare_parameter("angle_min", rclcpp::ParameterValue(angle_min));
    get_parameter("angle_min", angle_min);

    declare_parameter("angle_max", rclcpp::ParameterValue(angle_max));
    get_parameter("angle_max", angle_max);

    declare_parameter("angle_increment", rclcpp::ParameterValue(angle_increment));
    get_parameter("angle_increment", angle_increment);

    declare_parameter("time_increment", rclcpp::ParameterValue(time_increment));
    get_parameter("time_increment", time_increment);

    declare_parameter("scan_time", rclcpp::ParameterValue(scan_time));
    get_parameter("scan_time", scan_time);

    declare_parameter("range_min", rclcpp::ParameterValue(range_min));
    get_parameter("range_min", range_min);

    declare_parameter("range_max", rclcpp::ParameterValue(range_max));
    get_parameter("range_max", range_max);

    declare_parameter("lidar_noise", rclcpp::ParameterValue(lidar_noise));
    get_parameter("lidar_noise", lidar_noise);

    gauss_dist_vel_noise = std::normal_distribution<>{0.0, (input_noise)};
    gauss_dist_obstacle_noise = std::normal_distribution<>{0.0, (basic_sensor_variance)};
    gauss_dist_lidar_noise = std::normal_distribution<>{0.0, (lidar_noise)};
    unif_dist = std::uniform_real_distribution<>{-slip_fraction, slip_fraction};
    collided = false;

    tf2_rostf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    timestep_pub_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
    timer_ =
      create_wall_timer(
      std::chrono::milliseconds(int(1.0 / rate * 1000)),
      std::bind(&Nusim::timer_callback, this));
    reset_srv_ = create_service<std_srvs::srv::Empty>(
      "~/reset",
      std::bind(&Nusim::reset, this, std::placeholders::_1, std::placeholders::_2));
    teleport_srv_ = create_service<nusim::srv::Teleport>(
      "~/teleport",
      std::bind(&Nusim::teleport, this, std::placeholders::_1, std::placeholders::_2));
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacles", 10);
    wheel_cmd_sub = create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
      "/wheel_cmd", 10, std::bind(
        &Nusim::wheel_cmd_callback, this,
        std::placeholders::_1));
    red_path_pub = create_publisher<nav_msgs::msg::Path>("~/redpath", 10);
    sd_pub = create_publisher<nuturtlebot_msgs::msg::SensorData>("~/sensor_data", 10);
    laser_scan_pub = create_publisher<sensor_msgs::msg::LaserScan>("~/sim_lidar", 1000);
    five_hz_timer = 
      create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&Nusim::fake_sensor_timer, this));
    fake_sensor_pub = create_publisher<visualization_msgs::msg::MarkerArray>("~/fake_sensor", 1000);
  }

private:
  size_t timestep;
  double rate, x0, y0, z0, theta0, cyl_radius, cyl_height, init_x, init_y, init_z;
  std::vector<double> obs_x, obs_y;
  double x_length, y_length, input_noise, slip_fraction, basic_sensor_variance, max_range, collision_radius, encoder_ticks_per_rad, motor_cmd_per_rad_sec;
  bool draw_only, collided;
  double angle_min, angle_max, angle_increment, time_increment, scan_time, range_min, range_max, lidar_noise;
  std_msgs::msg::UInt64 ts;
  rclcpp::TimerBase::SharedPtr timer_, five_hz_timer;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_, fake_sensor_pub;
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_sub;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sd_pub;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_srv_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf2_rostf_broadcaster_;
  geometry_msgs::msg::TransformStamped t;
  tf2::Quaternion q;
  int marker_id = 0;
  visualization_msgs::msg::MarkerArray all_cyl, measured_cyl;
  rclcpp::Time marker_time;
  std::vector<double> wheel_velocities{0.0, 0.0};
  // double max_rot_vel = 2.84; // from turtlebot3 website, in rad/s
  turtlelib::DiffDrive redbot{0.0, 0.0, x0, y0, theta0};
  nuturtlebot_msgs::msg::SensorData sd = nuturtlebot_msgs::msg::SensorData();
  visualization_msgs::msg::Marker walls_x, walls_y;
  double wall_thickness = 0.1;
  double tolerance = 0.01;
  nav_msgs::msg::Path red_path;
  geometry_msgs::msg::PoseStamped current_point;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr red_path_pub;
  std::normal_distribution<> gauss_dist_vel_noise, gauss_dist_obstacle_noise, gauss_dist_lidar_noise;
  std::uniform_real_distribution<> unif_dist;
  double relative_x, relative_y, a, b, c, disc, x_int1, x_int2, y_int1, y_int2, meas_samp_x, meas_samp_y, d1, d2, theta_collision;
  sensor_msgs::msg::LaserScan fake_lidar;
  std::vector<std::vector<double>> poss_collision;
  std::vector<float> range_lidar;
  int count = 0;

  /// \brief Timer callback
  void timer_callback()
  {
    if (timestep == 0) {
      check_len();
      marker_time = get_clock()->now();
      create_all_cylinders();
      create_walls();
      all_cyl.markers.push_back(walls_x);
      update_all_cylinders();
      all_cyl.markers.push_back(walls_y);

      init_x = x0;
      init_y = y0;
      init_z = z0;
      sd.left_encoder = 0.0;
      sd.right_encoder = 0.0;

      red_path.header.frame_id = "nusim/world";
      current_point.header.frame_id = "nusim/world";

      fake_lidar.header.frame_id = "red/base_scan";
      fake_lidar.header.stamp = get_clock()->now();
      fake_lidar.angle_min = angle_min;
      fake_lidar.angle_max = angle_max;
      fake_lidar.angle_increment = angle_increment;
      fake_lidar.time_increment = time_increment;
      fake_lidar.scan_time = scan_time;
      fake_lidar.range_min = range_min;
      fake_lidar.range_max = range_max;
      fake_lidar.ranges.resize((angle_max - angle_min)/angle_increment + 1);
    }
    ts.data = timestep;
    timestep_pub_->publish(ts);
    if (draw_only) {
      t.header.stamp = get_clock()->now();
      t.header.frame_id = "nusim/world";
      t.child_frame_id = "red/base_footprint";
      t.transform.translation.x = x0;
      t.transform.translation.y = y0;
      t.transform.translation.z = z0;
      q.setRPY(0, 0, theta0);
      t.transform.rotation.x = q.x();
      t.transform.rotation.y = q.y();
      t.transform.rotation.z = q.z();
      t.transform.rotation.w = q.w();
      tf2_rostf_broadcaster_->sendTransform(t);

      if (timestep % 50 == 0) {
        red_path.header.stamp = get_clock()->now();
        current_point.header.stamp = get_clock()->now();
        current_point.pose.position.x = x0;
        current_point.pose.position.y = y0;
        current_point.pose.position.z = z0;
        red_path.poses.push_back(current_point);
        red_path_pub->publish(red_path);
      }
    }
    timestep += 1;
    marker_pub_->publish(all_cyl);
    update_sd();
    sd_pub->publish(sd);
    if (timestep%40 == 0) {
      // RCLCPP_INFO(get_logger(), "red robot state: %.3f, %.3f, %.3f", redbot.getCurrentConfig().at(2), redbot.getCurrentConfig().at(0), redbot.getCurrentConfig().at(1));
    }
  }

  /// \brief timer callback at 5 hz for a fake sensor
  void fake_sensor_timer() {
    collided = false;
    cylinders_as_measured();
    simulate_lidar();
    fake_sensor_pub->publish(measured_cyl);
    fake_lidar.header.stamp = get_clock()->now() - std::chrono::milliseconds(200); //- std::chrono::milliseconds(fake_lidar.ranges.size()*10/5);
    laser_scan_pub->publish(fake_lidar);
  }

  /// \brief Wheel_cmd subscription
  void wheel_cmd_callback(nuturtlebot_msgs::msg::WheelCommands cmd)
  {
    wheel_velocities.at(0) = cmd.left_velocity / motor_cmd_per_rad_sec; // * max_rot_vel / 265.0;
    wheel_velocities.at(1) = cmd.right_velocity / motor_cmd_per_rad_sec; // * max_rot_vel / 265.0;
    if (wheel_velocities.at(0) != 0.0 && wheel_velocities.at(1) != 0.0) {
      wheel_velocities.at(0) += gauss_dist_vel_noise(get_random());
      wheel_velocities.at(1) += gauss_dist_vel_noise(get_random());
    }
  }

  /// \brief updating sensor data
  void update_sd()
  {
    sd.stamp.sec = floor(get_clock()->now().nanoseconds() * 1e-9);
    sd.stamp.nanosec = get_clock()->now().nanoseconds() - sd.stamp.sec * 1e9;
    // here is where we'd add the slip fraction randomness
    double left_new_pos = wheel_velocities.at(0) * (1.0 / rate);
    double right_new_pos = wheel_velocities.at(1) * (1.0 / rate);
    std::vector<double> phi_current = redbot.getWheelPos();
    redbot.fkinematics(
      std::vector<double>{left_new_pos, right_new_pos});
    for (size_t loop = 0; loop < measured_cyl.markers.size(); loop++) {
      if (measured_cyl.markers.at(loop).action != 2 && 
          (distance(0.0, 0.0, measured_cyl.markers.at(loop).pose.position.x, measured_cyl.markers.at(loop).pose.position.y) <= (collision_radius + cyl_radius)) &&
           !collided) {
        RCLCPP_INFO(get_logger(), "collided!");
        turtlelib::Vector2D vec{(y0 - measured_cyl.markers.at(loop).pose.position.y), (x0 - measured_cyl.markers.at(loop).pose.position.x)};
        theta_collision = std::atan2((y0 + measured_cyl.markers.at(loop).pose.position.y), (x0 + measured_cyl.markers.at(loop).pose.position.x));
        x0 = x0 - (collision_radius + cyl_radius - distance(0.0, 0.0, measured_cyl.markers.at(loop).pose.position.x, measured_cyl.markers.at(loop).pose.position.y)) * cos(theta_collision);
        y0 = y0 - (collision_radius + cyl_radius - distance(0.0, 0.0, measured_cyl.markers.at(loop).pose.position.x, measured_cyl.markers.at(loop).pose.position.y)) * sin(theta_collision);
        redbot.setCurrentConfig(std::vector<double>{x0, y0, theta0}); // robot doesn't move, but wheels still updated
        collided = true;
      }
    }
    if (!collided) {
      x0 = redbot.getCurrentConfig().at(0);
      y0 = redbot.getCurrentConfig().at(1);
      theta0 = redbot.getCurrentConfig().at(2);
    }
    std::vector<double> phiprime_noise{
      wheel_velocities.at(0) * (1 + unif_dist(get_random())) * (1.0/rate),
      wheel_velocities.at(1) * (1 + unif_dist(get_random())) * (1.0/rate)
    };
    redbot.setWheelPos(std::vector<double>{phiprime_noise.at(0) + redbot.getWheelPos().at(0), phiprime_noise.at(1) + redbot.getWheelPos().at(1)});
    sd.left_encoder += (left_new_pos * encoder_ticks_per_rad);
    sd.right_encoder += (right_new_pos * encoder_ticks_per_rad);
  }

  /// \brief helper function to generate measured cylinders
  void cylinders_as_measured() {
    measured_cyl = all_cyl;
    relative_x = redbot.getCurrentConfig().at(0);
    relative_y = redbot.getCurrentConfig().at(1);
    for (size_t loop = 0; loop < all_cyl.markers.size(); loop++) {
      if (distance(relative_x, relative_y, all_cyl.markers.at(loop).pose.position.x, all_cyl.markers.at(loop).pose.position.y) > max_range || 
          distance(relative_x, relative_y, all_cyl.markers.at(loop).pose.position.x, all_cyl.markers.at(loop).pose.position.y) < range_min) {
        measured_cyl.markers.at(loop).action = visualization_msgs::msg::Marker::DELETE;
      }
      else {
        measured_cyl.markers.at(loop).header.stamp = get_clock()->now();
        relative_x = all_cyl.markers.at(loop).pose.position.x - redbot.getCurrentConfig().at(0);
        relative_y = all_cyl.markers.at(loop).pose.position.y - redbot.getCurrentConfig().at(1);
        measured_cyl.markers.at(loop).pose.position.x = relative_x + gauss_dist_obstacle_noise(get_random());
        measured_cyl.markers.at(loop).pose.position.y = relative_y + gauss_dist_obstacle_noise(get_random());
        measured_cyl.markers.at(loop).color.g = 172.0 / 256.0; // change color to yellow
      }
      if (loop >= (all_cyl.markers.size() - 2)) {
        measured_cyl.markers.at(loop).action = visualization_msgs::msg::Marker::DELETE;
      }
    }
  }

  /// \brief helper function to simulate lidar values
  void simulate_lidar() {
    fake_lidar.ranges.clear();
    for (double sample = angle_min; sample < angle_max; sample+=angle_increment) {
      range_lidar.clear();
      meas_samp_x = x0 + range_max*cos(theta0 + sample);
      meas_samp_y = y0 + range_max*sin(theta0 + sample);
      // check dist to walls
      for (size_t check = 0; check < (walls_x.points.size() + walls_y.points.size()); check++) {
        range_lidar.push_back(
          check_incidence(x0, y0, meas_samp_x, meas_samp_y, poss_collision.at(check).at(0), poss_collision.at(check).at(1), 0.001));
      }
      // check dist to obstacles
      for (size_t check = (walls_x.points.size() + walls_y.points.size()); check < poss_collision.size(); check++) {
        range_lidar.push_back(
          check_incidence(x0, y0, meas_samp_x, meas_samp_y, poss_collision.at(check).at(0), poss_collision.at(check).at(1), cyl_radius));
      }
      range_lidar.at(0) = *std::min_element(range_lidar.begin(), range_lidar.end());
      if (range_lidar.at(0) < max_range){
        fake_lidar.ranges.push_back(range_lidar.at(0));
      }
      else {
        fake_lidar.ranges.push_back(0.0);
      }
    }
  }

  /// \brief helper function to determine collision
  /// \param Rx - current point, x
  /// \param Ry - current point, y
  /// \param Mx - target point, x
  /// \param My - target point, y
  /// \param obs_x - obstacle x
  /// \param obs_y - obstacle y
  /// \param r - obstacle radius
  /// \return distance to obstacle
  float check_incidence(double Rx, double Ry, double Mx, double My, double obs_x, double obs_y, double r) {
    a = 1 + (My-Ry)/(Mx-Rx)*(My-Ry)/(Mx-Rx);
    b = 2 * (-obs_x + (-(My-Ry)/(Mx-Rx)*Rx + Ry - obs_y) * (My-Ry)/(Mx-Rx));
    c = (obs_x*obs_x - r*r + ((My-Ry)/(Mx-Rx)*Rx - Ry + obs_y)*((My-Ry)/(Mx-Rx)*Rx - Ry + obs_y));
    disc = b*b - 4*a*c;

    // incident
    if (disc >= 0) {
      // tangent
      x_int1 = (-b + sqrt(disc))/(2*a);
      y_int1 = (My-Ry)/(Mx-Rx) * (x_int1-Rx) + Ry;
      d1 = distance(Rx, Ry, x_int1, y_int1);
      if (disc >= 0.001) {
        x_int2 = (-b - sqrt(disc))/(2*a);
        y_int2 = (My-Ry)/(Mx-Rx) * (x_int2-Rx) + Ry;
        d2 = distance(Rx, Ry, x_int2, y_int2);
        if (d2 < max_range) {
          if (d2 < d1 && ((x_int2-Rx)/(Mx-Rx) > 0) && ((y_int2-Ry)/(My-Ry) > 0)) {
            return d2;
          }
          else if (d1 < max_range && ((x_int1-Rx)/(Mx-Rx) > 0) && ((y_int1-Ry)/(My-Ry) > 0)) {
            return d1;
          }
        }
      }
      else {
        if (((x_int1-Rx)/(Mx-Rx) > 0) && ((y_int1-Ry)/(My-Ry) > 0)) {
          return d1;
        }
      }
    }
    return 10*max_range;
  }

  /// \brief Reset timestep by listening to reset service
  void reset(
    const std_srvs::srv::Empty::Request::SharedPtr,
    const std_srvs::srv::Empty::Response::SharedPtr)
  {
    RCLCPP_INFO(get_logger(), "Resetting...");
    timestep = 0;
    x0 = init_x;
    y0 = init_y;
    z0 = init_z;
  }

  /// \brief Teleport robot based on teleport service
  /// \param request - nusim::srv::Teleport type, contains x, y, and theta
  void teleport(
    nusim::srv::Teleport::Request::SharedPtr request,
    nusim::srv::Teleport::Response::SharedPtr)
  {
    RCLCPP_INFO(get_logger(), "Teleport service...");
    x0 = request->x;
    y0 = request->y;
    theta0 = request->theta;
  }

  void create_walls()
  {
    walls_x.header.frame_id = "nusim/world";
    walls_x.header.stamp = marker_time;
    walls_x.type = 6;
    walls_x.id = marker_id;
    walls_x.action = 0;
    walls_x.scale.x = 0.1; // x_length; // 2 really long cubes in x-dir
    walls_x.scale.y = wall_thickness;
    walls_x.scale.z = 0.25;
    walls_x.color.b = 1.0; // walls are blue
    walls_x.color.a = 1.0;
    for (double start = (-x_length/2.0); start < (x_length/2.0); start += walls_x.scale.x) {
      walls_x.points.push_back(
        geometry_msgs::build<geometry_msgs::msg::Point>().
        x(start).y(y_length/2.0).z(walls_x.scale.z / 2.0));
      walls_x.points.push_back(
        geometry_msgs::build<geometry_msgs::msg::Point>().
        x(start).y(-y_length/2.0).z(walls_x.scale.z / 2.0));
    }
    marker_id += 1;

    walls_y.header.frame_id = "nusim/world";
    walls_y.header.stamp = marker_time;
    walls_y.type = 6;
    walls_y.id = marker_id;
    walls_y.action = 0;
    walls_y.scale.x = wall_thickness;
    walls_y.scale.y = 0.1; // y_length; // 2 really long cubes in y-dir
    walls_y.scale.z = 0.25;
    walls_y.color.b = 1.0; // walls are blue
    walls_y.color.a = 1.0;
    for (double start = (-y_length/2.0); start < (y_length/2.0); start += walls_y.scale.y) {
      walls_y.points.push_back(
        geometry_msgs::build<geometry_msgs::msg::Point>().
        x(x_length/2.0).y(start).z(walls_y.scale.z / 2.0));
      walls_y.points.push_back(
        geometry_msgs::build<geometry_msgs::msg::Point>().
        x(-x_length/2.0).y(start).z(walls_y.scale.z / 2.0));
        // RCLCPP_INFO(get_logger(), "wall y points: %.3f, %.3f, %.3f", x_length/2.0, start, walls_y.scale.z / 2.0);
    }
    marker_id += 1;

    for (size_t loop = 0; loop < walls_x.points.size(); loop++) {
      poss_collision.push_back(std::vector<double>{walls_x.points.at(loop).x, walls_x.points.at(loop).y});
    }
    for (size_t loop = 0; loop < walls_y.points.size(); loop++) {
      poss_collision.push_back(std::vector<double>{walls_y.points.at(loop).x, walls_y.points.at(loop).y });
    }
  }

  /// \brief Create a single cylinder and add it to a MarkerArray
  /// \returns marker (cylinder)
  visualization_msgs::msg::Marker create_cylinder()
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "nusim/world";
    marker.header.stamp = marker_time;
    marker.id = marker_id;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.color.a = 1.0;
    marker.color.r = 202.0 / 256.0;
    marker.color.g = 52.0 / 256.0;
    marker.color.b = 51.0 / 256.0;
    all_cyl.markers.push_back(marker);

    return marker;
  }

  /// \brief Create all cylinders based on yaml
  void create_all_cylinders()
  {
    for (size_t loop = 0; loop < obs_x.size(); loop++) {
      visualization_msgs::msg::Marker cylinder = create_cylinder();
      marker_id += 1;
    }
  }

  /// \brief Update positions, timesteps, etc when publishing
  void update_all_cylinders()
  {
    for (size_t loop = 0; loop < obs_x.size(); loop++) {
      all_cyl.markers.at(loop).header.stamp = marker_time;
      all_cyl.markers.at(loop).pose.position.x = obs_x.at(loop);
      all_cyl.markers.at(loop).pose.position.y = obs_y.at(loop);
      all_cyl.markers.at(loop).pose.position.z = cyl_height / 2.0;
      all_cyl.markers.at(loop).scale.x = cyl_radius;
      all_cyl.markers.at(loop).scale.y = cyl_radius;
      all_cyl.markers.at(loop).scale.z = cyl_height;
      poss_collision.push_back(std::vector<double>{obs_x.at(loop), obs_y.at(loop)});
    }
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

  /// \brief Make sure the length of x and y coordinates are the same
  void check_len()
  {
    if (obs_x.size() == obs_y.size()) {
    } else {
      RCLCPP_INFO(get_logger(), "obstacle x and y list have differing lengths!");
      rclcpp::shutdown();
    }
  }

  /// \brief helper function to generate random variable
  /// \return random object
  std::mt19937 & get_random() // source: Elwin, Matt
  {
    // static variables inside a function are created once and persist for the remainder of the program
    static std::random_device rd{}; 
    static std::mt19937 mt{rd()};
    // we return a reference to the pseudo-random number genrator object. This is always the
    // same object every time get_random is called
    return mt;
  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusim>());
  rclcpp::shutdown();
  return 0;
}
