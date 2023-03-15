/// \file
/// \brief cluster landmarks from lidar
///
/// PARAMETERS:
///     rate (double): frequency of timer callback (hertz), defaults to 200
///     angle_increment (double): angle between consecutive readings from lidar (rad)
/// PUBLISHES:
///     ~/clusters (visualization_msgs::msg::MarkerArray): a way to visualize the clusters in rviz
/// SUBSCRIBES:
///     ~/sim_lidar (sensor_msgs::msg::LaserScan): simulated lidar data
/// SERVERS:
///     none
/// CLIENTS:
///     none

#include <chrono>
#include <cmath>
#include <random>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <armadillo>

using namespace std::chrono_literals;
using namespace arma;

class Landmarks : public rclcpp::Node
{
    public:
    Landmarks()
    : Node("landmarks"),
    rate(5.0),
    angle_increment(0.01745329238474369),
    max_range(0.5),
    cluster_count(6),
    marker_frame("red/base_footprint")
    {

        declare_parameter("rate", rclcpp::ParameterValue(rate));
        get_parameter("rate", rate);
        declare_parameter("angle_increment", rclcpp::ParameterValue(angle_increment));
        get_parameter("angle_increment", angle_increment);
        declare_parameter("max_range", rclcpp::ParameterValue(max_range));
        get_parameter("max_range", max_range);
        declare_parameter("cluster_count", rclcpp::ParameterValue(cluster_count));
        get_parameter("cluster_count", cluster_count);
        declare_parameter("marker_frame", rclcpp::ParameterValue(marker_frame));
        get_parameter("marker_frame", marker_frame);

        lidar_sub = create_subscription<sensor_msgs::msg::LaserScan>(
        "~/sim_lidar", 10, std::bind(
            &Landmarks::lidar_callback, this,
            std::placeholders::_1));
        // only use this on the actual turtlebot
        laser_sub = create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", rclcpp::SensorDataQoS(), std::bind(&Landmarks::lidar_callback, this, std::placeholders::_1));
        cluster_pub = create_publisher<visualization_msgs::msg::MarkerArray>("~/clusters", 1000);
        timer =
        create_wall_timer(
        std::chrono::milliseconds(int(1.0 / rate * 1000)),
        std::bind(&Landmarks::timer_callback, this));
    }
    private:
        double rate, angle_increment, max_range;
        size_t poss_obs;
        int cluster_count; // number of points before its considered a cluster
        std::string marker_frame;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub, laser_sub;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cluster_pub;
        rclcpp::TimerBase::SharedPtr timer;
        vec ranges = vec(359, arma::fill::zeros);
        std::vector<std::vector<float>> clusters;
        // mat clusters = mat(359, 359, -1*arma::fill::ones);
        int clust_check_count = 0;
        bool is_cluster = false;
        double tolerance = 0.15;
        visualization_msgs::msg::MarkerArray cluster_cyl;

        /// \brief Timer callback
        void timer_callback() {
            if (cluster_cyl.markers.size() < 15) {
                // create 10 cyl
                for (int size_check = 0; size_check < 15; size_check++) {
                    create_cylinder(size_check);
                }
            }
            if (clusters.size() > 0) {
                for (size_t index = 0; index < clusters.size(); index++) {
                    update_cylinder();
                }
                cluster_pub->publish(cluster_cyl);
            }
        }

        /// \brief subscription callback for lidar data 
        /// \param ls - lidar data received 
        void lidar_callback(sensor_msgs::msg::LaserScan ls) {
            ranges = arma::conv_to<arma::vec>::from(ls.ranges);
            cluster();
            // RCLCPP_ERROR_STREAM(get_logger(), "ranges: \n" << ranges);
        }

        /// \brief cluster points from ranges
        void cluster() {
            int cluster_row = 0;
            clusters.clear();
            for (size_t point = 0; point < (ranges.n_elem - 1); point++) {
                // are cluster_count (4) consecutive values nonzero?
                if (ranges(point) > 0.0 && ranges(point) < max_range) {
                    clust_check_count++;
                }
                else if (clust_check_count >= cluster_count) {
                    // there were at least cluster_count (4) consecutive values that were positive
                    size_t all_points = clust_check_count;
                    clust_check_count = 0;
                    // RCLCPP_INFO(get_logger(), "ind pt: %ld", +point);
                    for (size_t next_point = 1; next_point < (all_points - 1); next_point++) {
                        if ((point+next_point + 1) >= ranges.n_elem) {
                            next_point = all_points;
                        }
                        else if (std::abs(ranges(point+next_point) - ranges(point)) < tolerance) {
                            clust_check_count++;
                        }
                    }
                    // here were at least cluster_count (4) consecutive values that had the same distance as (+point)
                    if (clust_check_count >= (cluster_count - 1) && (point >= clust_check_count)) {
                        // RCLCPP_INFO(get_logger(), "cluster found: %ld", point);
                        // RCLCPP_INFO(get_logger(), "test 1.2.1: %d", clust_check_count);
                        // RCLCPP_ERROR_STREAM(get_logger(), "ranges: \n" << ranges);
                        clusters.push_back(std::vector<float>{(float) point - clust_check_count}); // the first element of a row is the angle value of where it was measured
                        for (int addMe = 1; addMe < clust_check_count; addMe++) {
                            if ((point - clust_check_count + addMe + 1) < ranges.n_elem) {
                                clusters.at(cluster_row).push_back(ranges(point - clust_check_count + addMe));
                            }
                        }
                        // RCLCPP_ERROR_STREAM(get_logger(), "cluster points: \n" << arma::conv_to<arma::vec>::from(clusters.at(cluster_row)));
                        // clusters.at(mark).at(index) * cos(clusters.at(mark).at(0) * angle_increment)).
                        //                                             y(clusters.at(mark).at(index) * sin(clusters.at(mark).at(0) * angle_increment)).
                        //                                             z(cluster_cyl.markers.at(mark).scale.z / 2.0));
                        point += clust_check_count; // skip the next clust_check points since we already know they're in this cluster
                        cluster_row++;
                    }
                    clust_check_count = 0;
                }
                else {
                    clust_check_count = 0;
                }
            }
            is_cluster = false;
            // it's possible that the scan wrapped around, and a cluster is formed from the last n number of points
            if ((abs(ranges(ranges.n_elem - 1)) < max_range) && (abs(ranges(0)) < max_range)) {
                std::vector<double> wrap_around;
                for (int loop = -cluster_count; loop < cluster_count; loop++) {
                    if (loop < 0) {
                        wrap_around.push_back(ranges(ranges.n_elem + loop));
                    }
                    else {
                        wrap_around.push_back(ranges(loop));
                    }
                }
                clust_check_count = 0;
                int first = 0;
                // RCLCPP_INFO(get_logger(), "points: %.2f, %.2f, %.2f", ranges(index+1), ranges(index), std::abs(ranges(index+1) - ranges(index)));
                for (size_t point = 0; point < (wrap_around.size() - 1); point++) {
                    if ((std::abs(wrap_around.at(point+1) - wrap_around.at(point)) < tolerance) && 
                        (wrap_around.at(point) > 0)) {
                        if (clust_check_count == 0) {
                            first = point;
                        }
                        clust_check_count++;
                    }
                }
                if (clust_check_count >= (cluster_count - 1)) {
                    // RCLCPP_INFO(get_logger(), "test wrap");
                    if (first < cluster_count) {
                        first = ranges.n_elem - 1 - first;
                    }
                    // RCLCPP_INFO(get_logger(), "cluster found at the end!");
                    clusters.push_back(std::vector<float>{(float) first}); // the first element of a row is the angle value of where it was measured
                    for (int point = 0; point < cluster_count; point++) {
                        if ((first + point + 1) == (int) ranges.n_elem) {
                            first = 0;
                        }
                        clusters.at(cluster_row).push_back(ranges(first + point));
                    }
                }
                else {
                    clust_check_count = 0;
                }
            }
        }

        /// \brief Create a single cylinder and add it to a MarkerArray
        /// \param mark - marker_id
        void create_cylinder(int mark)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = marker_frame;
            marker.header.stamp = get_clock()->now();
            marker.id = mark;
            marker.type = 6;
            marker.scale.x = 0.01;
            marker.scale.y = 0.01;
            marker.scale.z = 0.5;
            marker.color.a = 1.0;
            marker.color.r = 112.0 / 256.0;
            marker.color.g = 128.0 / 256.0;
            marker.color.b = 144.0 / 256.0;
            marker.action = visualization_msgs::msg::Marker::DELETE;
            cluster_cyl.markers.push_back(marker);
        }

        /// \brief update cylinder
        void update_cylinder() {
            for (size_t mark = 0; mark < cluster_cyl.markers.size(); mark++) {
                cluster_cyl.markers.at(mark).header.stamp = get_clock()->now();
                if (mark < clusters.size()) {
                    cluster_cyl.markers.at(mark).action = visualization_msgs::msg::Marker::ADD;
                    cluster_cyl.markers.at(mark).points = {};
                    for (size_t index = 1; index < clusters.at(mark).size(); index++) {
                        cluster_cyl.markers.at(mark).points.push_back(geometry_msgs::build<geometry_msgs::msg::Point>().
                                                                    x(clusters.at(mark).at(index) * cos(clusters.at(mark).at(0) * angle_increment)).
                                                                    y(clusters.at(mark).at(index) * sin(clusters.at(mark).at(0) * angle_increment)).
                                                                    z(cluster_cyl.markers.at(mark).scale.z / 2.0));
                    }
                }
                else {
                    cluster_cyl.markers.at(mark).action = visualization_msgs::msg::Marker::DELETE;
                }
            }
        }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Landmarks>());
  rclcpp::shutdown();
  return 0;
}
