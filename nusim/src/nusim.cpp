#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"

using namespace std::chrono_literals;

class Nusim : public rclcpp::Node
{
    public:
        Nusim():
            Node("nusim"),
            timestep(0)
        {
            double rate = 200.0;
            rcl_interfaces::msg::ParameterDescriptor rate_param_desc;
            rate_param_desc.name = "rate";
            rate_param_desc.type = 3; // rate is a double
            rate_param_desc.description = "simulation refresh rate (hz)";
            declare_parameter("rate", rclcpp::ParameterValue(rate), rate_param_desc); // defaults to 200
            get_parameter("rate", rate);
            timestep_pub_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
            timer_ = create_wall_timer(std::chrono::milliseconds(int(1.0/rate*1000)), std::bind(&Nusim::timer_callback, this));
            reset_srv_ = create_service<std_srvs::srv::Empty>(
                "~/reset", 
                std::bind(&Nusim::reset, this, std::placeholders::_1, std::placeholders::_2));
            // auto ts = std_msgs::msg::UInt64();
        }

    private:
        void timer_callback() {
            ts.data = timestep;
            // RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Publish count " << this->count_);
            // RCLCPP_INFO(get_logger(), "stuff: %ld", timestep);
            timestep_pub_->publish(ts);
            timestep += 1;
        }
        void reset(const std_srvs::srv::Empty::Request::SharedPtr request, 
                    const std_srvs::srv::Empty::Response::SharedPtr response) {
            RCLCPP_INFO(get_logger(), "Resetting...");
            timestep = 0;
        }
        size_t timestep;
        std_msgs::msg::UInt64 ts;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_pub_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Nusim>());
    rclcpp::shutdown();
    return 0;
}