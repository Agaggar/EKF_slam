#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "nusim/srv/teleport.hpp"

using namespace std::chrono_literals;

class Nusim : public rclcpp::Node
{
public:
  Nusim()
  : Node("nusim"),
    timestep(0)
  {
    double rate = 200.0;
    rcl_interfaces::msg::ParameterDescriptor rate_param_desc;
    rate_param_desc.name = "rate";
    rate_param_desc.type = 3;         // rate is a double
    rate_param_desc.description = "simulation refresh rate (hz)";
    declare_parameter("rate", rclcpp::ParameterValue(rate), rate_param_desc);         // defaults to 200
    get_parameter("rate", rate);
    double x0 = 0.0;
    rcl_interfaces::msg::ParameterDescriptor x0_param_desc;
    x0_param_desc.name = "x0";
    x0_param_desc.type = 3;         // x0 is a double
    x0_param_desc.description = "initial x0 position (m)";
    declare_parameter("x0", rclcpp::ParameterValue(x0), x0_param_desc);         // defaults to 0.0
    get_parameter("x0", x0);
    double y0 = 0.0;
    rcl_interfaces::msg::ParameterDescriptor y0_param_desc;
    y0_param_desc.name = "y0";
    y0_param_desc.type = 3;         // y0 is a double
    y0_param_desc.description = "initial y0 position (m)";
    declare_parameter("y0", rclcpp::ParameterValue(y0), y0_param_desc);         // defaults to 0.0
    get_parameter("y0", y0);
    double theta0 = 0.0;
    rcl_interfaces::msg::ParameterDescriptor theta0_param_desc;
    theta0_param_desc.name = "theta0";
    theta0_param_desc.type = 3;         // theta0 is a double
    theta0_param_desc.description = "initial theta value (rad)";
    declare_parameter("theta0", rclcpp::ParameterValue(theta0), theta0_param_desc);         // defaults to 0.0
    get_parameter("theta0", theta0);

    // need to declare all other parameters from yaml

    RCLCPP_INFO(get_logger(), "stuff: %f, %f, %f, %f", rate, x0, y0, theta0);
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
    // auto ts = std_msgs::msg::UInt64();
  }

private:
  size_t timestep = 0.0;
  double x0 = get_parameter_or("x0", 0.0);
  double y0 = get_parameter_or("y0", 0.0);
  double theta0 = get_parameter_or("theta0", 0.0);
  std_msgs::msg::UInt64 ts;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_srv_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf2_rostf_broadcaster_;
  geometry_msgs::msg::TransformStamped t;
  tf2::Quaternion q;

  void timer_callback()
  {
    ts.data = timestep;
    // RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Publish count " << this->count_);
    // RCLCPP_INFO(get_logger(), "stuff: %ld", timestep);
    timestep_pub_->publish(ts); \
    t.header.stamp = get_clock()->now();
    t.header.frame_id = "nusim/world";
    t.child_frame_id = "red/base_footprint";
    t.transform.translation.x = x0;
    t.transform.translation.y = y0;
    t.transform.translation.z = 0.0;
    q.setRPY(0, 0, theta0);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    tf2_rostf_broadcaster_->sendTransform(t);
    timestep += 1;
    // RCLCPP_INFO(get_logger(), "stuff: %f", t.transform.translation.x);
  }

  void reset(
    const std_srvs::srv::Empty::Request::SharedPtr request,
    const std_srvs::srv::Empty::Response::SharedPtr response)
  {
    RCLCPP_INFO(get_logger(), "Resetting...");
    timestep = 0;
  }

  void teleport(
    // std::shared_ptr<nusim::srv::Teleport::Request> request,
    // std::shared_ptr<nusim::srv::Teleport::Response> response)
    nusim::srv::Teleport::Request::SharedPtr request,
    nusim::srv::Teleport::Response::SharedPtr response)
  {
    RCLCPP_INFO(get_logger(), "Teleport service...");
    x0 = request->x;
    y0 = request->y;
    theta0 = request->theta;
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusim>());
  rclcpp::shutdown();
  return 0;
}
