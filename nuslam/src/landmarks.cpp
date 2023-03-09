/// \file
/// \brief cluster landmarks from lidar
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
///     ~/sim_lidar (sensor_msgs::msg::LaserScan): publish simulated lidar data
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
    rate(200.0),
    angle_increment(0.01745329238474369)
    {

        declare_parameter("rate", rclcpp::ParameterValue(rate));
        get_parameter("rate", rate);
        declare_parameter("angle_increment", rclcpp::ParameterValue(angle_increment));
        get_parameter("angle_increment", angle_increment);

        lidar_sub = create_subscription<sensor_msgs::msg::LaserScan>(
        "~/sim_lidar", 10, std::bind(
            &Landmarks::lidar_callback, this,
            std::placeholders::_1));
        cluster_pub = create_publisher<visualization_msgs::msg::MarkerArray>("~/clusters", 1000);
        timer =
        create_wall_timer(
        std::chrono::milliseconds(int(1.0 / rate * 1000)),
        std::bind(&Landmarks::timer_callback, this));
    }
    private:
        double rate, angle_increment;
        size_t poss_obs;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cluster_pub;
        rclcpp::TimerBase::SharedPtr timer;
        vec ranges = vec(359, arma::fill::zeros);
        std::vector<std::vector<float>> clusters;
        // mat clusters = mat(359, 359, -1*arma::fill::ones);
        int cluster_count = 3; // number of points before its considered a cluster
        int clust_check_count = 0;
        bool is_cluster = false;
        double tolerance = 0.1;
        visualization_msgs::msg::MarkerArray cluster_cyl;

        /// \brief Timer callback
        void timer_callback() {
            if (cluster_cyl.markers.size() < 10) {
                // create 10 cyl
                for (int size_check = 0; size_check < 10; size_check++) {
                    create_cylinder(size_check);
                }
            }
            if (clusters.size() > 0) {
                for (size_t index = 0; index < clusters.size(); index++) {
                    update_cylinder(index);
                }
                cluster_pub->publish(cluster_cyl);
            }
        }

        /// \brief subscription callback for lidar data 
        /// \param ls - lidar data received 
        void lidar_callback(sensor_msgs::msg::LaserScan ls) {
            ranges = conv_to<vec>::from(ls.ranges);
            cluster();
            // RCLCPP_ERROR_STREAM(get_logger(), "ranges: \n" << ranges);
        }

        /// \brief cluster points from ranges
        void cluster() {
            int cluster_row = 0;
            for (size_t index = 0; index < (ranges.n_elem - cluster_count); index++) {
                clust_check_count = 0;
                is_cluster = false;
                // are cluster_count (3) consecutive values nonzero?
                for (int point = 0; point < cluster_count; point++) {
                    // RCLCPP_INFO(get_logger(), "index: %ld", index);
                    if (ranges(index+point) > 0.0) {
                        clust_check_count++;
                    }
                }
                // are cluster_count (3) consecutive values close to a tolerance value?
                if (clust_check_count == 3) {
                    clust_check_count = 0;
                    // RCLCPP_INFO(get_logger(), "points: %.2f, %.2f, %.2f", ranges(index+1), ranges(index), std::abs(ranges(index+1) - ranges(index)));
                    for (int point = 0; point < (cluster_count - 1); point++) {
                        if (std::abs(ranges(index+point+1) - ranges(index+point)) < tolerance) {
                            clust_check_count++;
                        }
                    }
                    if (clust_check_count == 2) {
                        RCLCPP_INFO(get_logger(), "cluster found: %ld", index);
                        is_cluster = true;
                    }
                }
                if (is_cluster) {
                    clusters.push_back(std::vector<float>{(float) index}); // the first element of a row is the angle value of where it was measured
                    for (int point = 0; point < cluster_count; point++) {
                        clusters.at(cluster_row).push_back(ranges(index + point));
                    }
                    index += (cluster_count - 1); // skip the next 2 (+1) points
                    cluster_row++;
                }
            }
            is_cluster = false;
            // it's possible that the scan wrapped around, and a cluster is formed from either
            // the last 2 points and the first point, OR the last point and the first 2 points
            if ((ranges(ranges.n_elem - 1) != 0.0) && (ranges(0) != 0.0)) {
                // last point, and first 2 points
                if (std::abs(ranges(ranges.n_elem - 1) - ranges(0)) > tolerance) {
                    if (std::abs(ranges(1) - ranges(0)) > tolerance) {
                        is_cluster = true;
                    }
                }
                if (is_cluster) {
                    clusters.push_back(std::vector<float>{(float) ranges.n_elem - 1});
                    clusters.at(cluster_row).push_back(ranges(ranges.n_elem - 1));
                    clusters.at(cluster_row).push_back(ranges(0));
                    clusters.at(cluster_row).push_back(ranges(1));
                    cluster_row++;
                } // last 2 points, and first point
                else if (std::abs(ranges(ranges.n_elem - 2) - ranges(ranges.n_elem - 1)) > tolerance) {
                    if (std::abs(ranges(ranges.n_elem - 1) - ranges(0)) > tolerance) {
                        is_cluster = true;
                    }
                    if (is_cluster) {
                        clusters.push_back(std::vector<float>{(float) ranges.n_elem - 2});
                        clusters.at(cluster_row).push_back(ranges(ranges.n_elem - 2));
                        clusters.at(cluster_row).push_back(ranges(ranges.n_elem - 1));
                        clusters.at(cluster_row).push_back(ranges(0));
                        cluster_row++;
                    }
                }
            }
        }

        /// \brief Create a single cylinder and add it to a MarkerArray
        /// \param mark - marker_id
        void create_cylinder(int mark)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "nusim/world";
            marker.header.stamp = get_clock()->now();
            marker.id = mark;
            marker.type = 6;
            marker.scale.x = 0.03;
            marker.scale.y = 0.03;
            marker.scale.z = 0.5;
            marker.color.a = 1.0;
            marker.color.r = 112.0 / 256.0;
            marker.color.g = 128.0 / 256.0;
            marker.color.b = 144.0 / 256.0;
            cluster_cyl.markers.push_back(marker);
        }

        /// \brief update cylinder
        /// \param mark - marker id
        void update_cylinder(int mark) {
            cluster_cyl.markers.at(mark).action = visualization_msgs::msg::Marker::ADD;
            for (size_t index = 1; index < clusters.at(mark).size(); index++) {
                cluster_cyl.markers.at(mark).points.push_back(geometry_msgs::build<geometry_msgs::msg::Point>().
                                                              x(clusters.at(mark).at(index) * cos(clusters.at(mark).at(0) * angle_increment)).
                                                              y(clusters.at(mark).at(index) * sin(clusters.at(mark).at(0) * angle_increment)).
                                                              z(cluster_cyl.markers.at(mark).scale.z / 2.0));
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
