/// \file
/// \brief spin nusim node to simulat one robot and several obstacles
///
/// PARAMETERS:
///     rate (double): frequency of timer callback (hertz)
///     x0 (double): initial x position of the robot (m)
///     y0 (double): initial y position of the robot (m)
///     z0 (double): initial z position of the robot (m)
///     theta0 (double): initial heading of the robot (rad)
///     obstacles.x (std::vector<double>): x coordinates of obstacles (m)
///     obstacles.y (std::vector<double>): y coordinates of obstacles (m)
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
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "nusim/srv/teleport.hpp"
#include "nusim/msg/wheel_commands.hpp"
#include "nusim/msg/sensor_data.hpp"
#include "turtlelib/diff_drive.hpp"
#include "builtin_interfaces/msg/time.hpp"

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
    y_length(2.5)
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
    wheel_cmd_sub = create_subscription<nusim::msg::WheelCommands>("/wheel_cmd", 10, std::bind(&Nusim::wheel_cmd_callback, this, std::placeholders::_1));
    sd_pub = create_publisher<nusim::msg::SensorData>("/sensor_data", 10);
  }

private:
  size_t timestep;
  double rate, x0, y0, z0, theta0, cyl_radius, cyl_height, init_x, init_y, init_z;
  std::vector<double> obs_x;
  std::vector<double> obs_y;
  double x_length, y_length;
  std_msgs::msg::UInt64 ts;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<nusim::msg::WheelCommands>::SharedPtr wheel_cmd_sub;
  rclcpp::Publisher<nusim::msg::SensorData>::SharedPtr sd_pub;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_srv_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf2_rostf_broadcaster_;
  geometry_msgs::msg::TransformStamped t;
  tf2::Quaternion q;
  int marker_id = 0;
  visualization_msgs::msg::MarkerArray all_cyl;
  rclcpp::Time marker_time;
  std::vector<double> wheel_velocities{0.0, 0.0};
  double max_rot_vel = 2.84; // from turtlebot3 website, in rad/s
  turtlelib::DiffDrive redbot{0.0, 0.0, x0, y0, theta0};
  nusim::msg::SensorData sd = nusim::msg::SensorData();
  double encoder_ticks_per_rad = 651.8986;
  visualization_msgs::msg::Marker walls_x, walls_y;
  double wall_thickness = 0.1;

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
    }
    ts.data = timestep;
    timestep_pub_->publish(ts);
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
    timestep += 1;
    marker_pub_->publish(all_cyl);
    update_sd();
    sd_pub->publish(sd);
  }

  /// \brief Wheel_cmd subscription
  void wheel_cmd_callback(nusim::msg::WheelCommands cmd) {
    wheel_velocities.at(0) = cmd.left_velocity * max_rot_vel / 265.0;
    wheel_velocities.at(1) = cmd.right_velocity * max_rot_vel / 265.0;
  }

  /// \brief updating sensor data
  void update_sd() {
    sd.stamp.sec = floor(get_clock()->now().nanoseconds()/1e-9);
    sd.stamp.nanosec = get_clock()->now().nanoseconds() - sd.stamp.sec * 1e9;
    double left_new_pos = wheel_velocities.at(0) * (1.0 / rate);
    double right_new_pos = wheel_velocities.at(1) * (1.0 / rate);
    redbot.fkinematics(std::vector<double>{left_new_pos, right_new_pos});
    sd.left_encoder = floor(left_new_pos * encoder_ticks_per_rad);
    sd.right_encoder = floor(right_new_pos * encoder_ticks_per_rad);
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
    // for (size_t loop = 0; loop < obs_x.size(); loop++) {
    //   all_cyl.markers.pop_back();
    // }
  }

  /// \brief Teleport robot based on teleport service
  ///
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

  void create_walls() {
    walls_x.header.frame_id = "nusim/world";
    walls_x.header.stamp = marker_time;
    walls_x.type = 6;
    walls_x.id = marker_id;
    walls_x.action = 0;
    walls_x.scale.x = x_length; // 2 really long cubes in x-dir
    walls_x.scale.y = wall_thickness;
    walls_x.scale.z = 0.25;
    walls_x.color.b = 1.0; // walls are blue
    walls_x.color.a = 1.0;
    std::vector<geometry_msgs::msg::Point> points;
    points.push_back(
      geometry_msgs::build<geometry_msgs::msg::Point>().
      x(0.0).y(y_length/2.0 + wall_thickness/2.0).z(walls_x.scale.z/2.0));
    points.push_back(
      geometry_msgs::build<geometry_msgs::msg::Point>().
      x(0.0).y(-y_length/2.0 - wall_thickness/2.0).z(walls_x.scale.z/2.0));
    marker_id += 1;
    walls_x.points = points;
    
    
    points.pop_back();
    points.pop_back(); // empty points to use for walls_y
    walls_y.header.frame_id = "nusim/world";
    walls_y.header.stamp = marker_time;
    walls_y.type = 6;
    walls_y.id = marker_id;
    walls_y.action = 0;
    walls_y.scale.x = wall_thickness;
    walls_y.scale.y = y_length; // 2 really long cubes in y-dir
    walls_y.scale.z = 0.25;
    walls_y.color.b = 1.0; // walls are blue
    walls_y.color.a = 1.0;
    points.push_back(
      geometry_msgs::build<geometry_msgs::msg::Point>().
      x(x_length/2.0 + wall_thickness/2.0).y(0.0).z(walls_y.scale.z/2.0));
    points.push_back(
      geometry_msgs::build<geometry_msgs::msg::Point>().
      x(-x_length/2.0 - wall_thickness/2.0).y(0.0).z(walls_y.scale.z/2.0));
    marker_id += 1;
    walls_y.points = points;
  }

  /// \brief Create a single cylinder and add it to a MarkerArray
  ///
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
      all_cyl.markers[loop].header.stamp = marker_time;
      all_cyl.markers[loop].pose.position.x = obs_x[loop];
      all_cyl.markers[loop].pose.position.y = obs_y[loop];
      all_cyl.markers[loop].pose.position.z = cyl_height / 2.0;
      all_cyl.markers[loop].scale.x = cyl_radius;
      all_cyl.markers[loop].scale.y = cyl_radius;
      all_cyl.markers[loop].scale.z = cyl_height;
    }
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
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusim>());
  rclcpp::shutdown();
  return 0;
}
