#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"

using namespace std::chrono_literals;

class Nusim : public rclcpp::Node
{
    public:
        Nusim():
            Node("nusim"),
            timestep(0)
        {
            declare_parameter("rate", 200); // defaults to 200
            timestep_pub_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
            timer_ = create_wall_timer(std::chrono::milliseconds(5ms), std::bind(&Nusim::timer_callback, this));
            // auto ts = std_msgs::msg::UInt64();
        }

    private:
        void timer_callback()
        {
            ts.data = timestep;
            // RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Publish count " << this->count_);
            // RCLCPP_INFO(get_logger(), "stuff: %ld", timestep);
            timestep_pub_->publish(ts);
            timestep += 1;
        }
        size_t timestep;
        std_msgs::msg::UInt64 ts;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_pub_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Nusim>());
    rclcpp::shutdown();
    return 0;
}