#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"

class Nusim : public rclcpp::Node
{
    public:
        Nusim():
            Node("nusim"),
            count_(0)
        {
            timer = create_wall_timer(std::chrono::duration(5ms), std::bind(&Nusim::timer_callback, this));
            // rclcpp::create_timer()
        }

    private:
        void timer_callback()
        {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Publish count " << this->count_);
            count_ += 1;
        }
        size_t count_;
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Nusim>());
    rclcpp::shutdown();
    return 0;
}