/// \file
/// \brief circle fitting algorithm (from https://nu-msr.github.io/navigation_site/lectures/circle_fit.html)
///
/// PARAMETERS:
///     rate (double): frequency of timer callback (hertz), defaults to 200
/// PUBLISHES:
///     none
/// SUBSCRIBES:
///     ~/clusters (visualization_msgs::msg::MarkerArray): clusters in rviz
/// SERVERS:
///     none
/// CLIENTS:
///     none

#include <chrono>
#include <cmath>
#include <random>
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "turtlelib/rigid2d.hpp"
#include <armadillo>
#include <iostream>

using namespace std::chrono_literals;
using namespace arma;

class CircleFit : public rclcpp::Node {
    public:
    CircleFit()
    : Node("circle_fit"),
    rate(5.0),
    cyl_radius(0.038),
    max_range(0.5)
    {
        declare_parameter("rate", rclcpp::ParameterValue(rate));
        get_parameter("rate", rate);
        declare_parameter("obstacles.r", rclcpp::ParameterValue(cyl_radius));
        get_parameter("obstacles.r", cyl_radius);
        declare_parameter("max_range", rclcpp::ParameterValue(max_range));
        get_parameter("max_range", max_range);

        cluster_sub = create_subscription<visualization_msgs::msg::MarkerArray>(
            "~/clusters", 100, std::bind(
                &CircleFit::cluster_callback, this, std::placeholders::_1)
            );
        circle_cluster_pub = create_publisher<visualization_msgs::msg::MarkerArray>(
            "landmarks/circle_clusters", 100);
        timer = create_wall_timer(std::chrono::milliseconds(int(1.0 / rate * 1000)),
                                  std::bind(&CircleFit::timer_callback, this));
    }
    private:
        double rate, cyl_radius, max_range;
        rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr cluster_sub;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr circle_cluster_pub;
        rclcpp::TimerBase::SharedPtr timer;
        mat clusters, data_points, bigZ, bigM, bigH, bigH_inv;
        arma::vec x_coor, y_coor, zi, bigA, angles;
        arma::uvec indices;
        std::vector<double> means, circle, statistics;
        double zbar, sum, rmse, mean_thresh = 0.1, stddev_thresh = 0.1;
        turtlelib::Vector2D P1, P2;
        visualization_msgs::msg::MarkerArray actual_circles;
        size_t timestep = 0;
        int marker_id = 0;

        /// \brief Timer callback 
        void timer_callback() {
            if (timestep == 0) {
                // assuming never more than 15 clusters at once
                for (size_t len = 0; len < 15; len++) {
                    actual_circles.markers.push_back(createCircle());
                    actual_circles.markers.at(len).id = marker_id;
                    marker_id++;
                }
            }
            circle_cluster_pub->publish(actual_circles);
            // if (actual_circles.markers.size() > 0) {
            // }
            timestep++;
        }

        visualization_msgs::msg::Marker createCircle() {
            visualization_msgs::msg::Marker circleAdd;
            circleAdd.action = 2;
            circleAdd.color.r = 75.0 / 256.0;
            circleAdd.color.g = 3.0 / 256.0;
            circleAdd.color.r = 132.0 / 256.0;
            circleAdd.type = visualization_msgs::msg::Marker::CYLINDER;
            // can put cyl_radius, cyl_height
            circleAdd.scale.x = cyl_radius; // circle.at(2);
            circleAdd.scale.y = cyl_radius; //circle.at(2);
            circleAdd.scale.z = 0.25;
            circleAdd.pose.position.z = circleAdd.scale.z / 2.0;
            return circleAdd;
        }

        /// \brief subscription callback for clusters
        /// \param clus - marker array from topic
        void cluster_callback(visualization_msgs::msg::MarkerArray clus) {
            for (size_t len = 0; len < actual_circles.markers.size(); len++) {
                actual_circles.markers.at(len).action = 2;
            }
            // marker_id = 0;
            for (size_t loop = 0; loop < clus.markers.size(); loop++) {
                clus.markers.at(loop).header.stamp = rclcpp::Time(0);
                if (clus.markers.at(loop).action == 0 && clus.markers.at(loop).points.size() >= 4 && actual_circles.markers.size() > 0) {
                    x_coor.zeros(clus.markers.at(loop).points.size());
                    y_coor.zeros(clus.markers.at(loop).points.size());
                    for (size_t index = 0; index < clus.markers.at(loop).points.size(); index++) {
                        x_coor(index) = clus.markers.at(loop).points.at(index).x;
                        y_coor(index) = clus.markers.at(loop).points.at(index).y;
                    }
                    means = computeMean(x_coor, y_coor);
                    // RCLCPP_INFO(get_logger(), "xMean: %.7f, yMean: %.7f", means.at(0), means.at(1));
                    data_points = arma::join_rows(x_coor, y_coor);
                    // test 1
                    // data_points = {{1, 7}, {2, 6}, {5, 8}, {7, 7}, {9, 5}, {3, 7}};
                    // // test 2
                    // data_points = {{-1,0}, {-0.3,-0.06}, {0.3, 0.1}, {1, 0}};
                    // means = computeMean(data_points.col(0), data_points.col(1));
                    // RCLCPP_ERROR_STREAM(get_logger(), "data: \n" << data_points);
                    x_coor = shiftPoints(data_points, means).col(0);
                    y_coor = shiftPoints(data_points, means).col(1);
                    zi = distToCentroid(shiftPoints(data_points, means));
                    zbar = findMean(zi);
                    bigZ = formDataMat(zi, x_coor, y_coor);
                    // RCLCPP_ERROR_STREAM(get_logger(), "capital Z: \n" << bigZ);
                    // bigM = formMoment(bigZ);
                    // RCLCPP_ERROR_STREAM(get_logger(), "bigM: \n" << bigM);
                    bigH = formConstraint(zbar);
                    // RCLCPP_ERROR_STREAM(get_logger(), "bigH: \n" << bigH);
                    bigH_inv = bigH.i();
                    bigA = computeA(bigZ, bigH);
                    // RCLCPP_ERROR_STREAM(get_logger(), "bigA: \n" << bigA);
                    circle = circleEq(bigA);
                    rmse = RMSE(circle, x_coor, y_coor);
                    // #TODO: radius filtering and/or statistic filtering to pick a circle
                    statistics = computeStats(data_points.col(0), data_points.col(1));
                    // RCLCPP_ERROR_STREAM(get_logger(), "data: \n" << arma::join_rows(x_coor, y_coor));
                    // shiftPoints(data_points, means).save("data.txt", raw_ascii);
                    // RCLCPP_INFO(get_logger(), "%ld: center: (%.4f, %.4f) R: %.4f", loop, circle.at(0) + means.at(0), circle.at(1) + means.at(1), circle.at(2));
                    RCLCPP_INFO(get_logger(), "cluster %d: center: (%.4f, %.4f) R: %.4f", actual_circles.markers.at(loop).id, circle.at(0) + means.at(0), circle.at(1) + means.at(1), circle.at(2));
                    // if (distance(circle.at(0) + means.at(0), circle.at(1) + means.at(1)) > .1) {
                    if (circle.at(2) < 0.1 && distance(circle.at(0) + means.at(0), circle.at(1) + means.at(1)) > .1
                        && distance(circle.at(0) + means.at(0), circle.at(1) + means.at(1)) < max_range) {
                    // if ((abs(statistics.at(0) - turtlelib::PI) <= mean_thresh) && (statistics.at(1) < stddev_thresh)) {
                        ////// CIRCLE FOUND
                        // RCLCPP_ERROR_STREAM(get_logger(), "data: \n" << data_points);
                        RCLCPP_INFO(get_logger(), "%d: center: (%.4f, %.4f) R: %.4f", actual_circles.markers.at(loop).id, circle.at(0) + means.at(0), circle.at(1) + means.at(1), circle.at(2));
                        actual_circles.markers.at(loop).header.stamp = get_clock()->now();
                        actual_circles.markers.at(loop).action = 0;
                        actual_circles.markers.at(loop).header.frame_id = clus.markers.at(loop).header.frame_id;
                        actual_circles.markers.at(loop).pose.position.x = circle.at(0) + means.at(0);
                        actual_circles.markers.at(loop).pose.position.y = circle.at(1) + means.at(1);
                        // actual_circles.markers.push_back(circleAdd);
                        // RCLCPP_INFO(get_logger(), "mean: %.4f, std: %.4f", turtlelib::rad2deg(statistics.at(0)), statistics.at(1));
                        // RCLCPP_ERROR_STREAM(get_logger(), "circle: " << (circle.at(0) + means.at(0)) << ", " << (circle.at(1) + means.at(1)) << ", " << sqrt(circle.at(2)));
                    }
                    else {
                        actual_circles.markers.at(loop).action = 2;
                    }
                }
            }
        }

        /// @brief find mean of two column vectors
        /// @param x - x coordinates
        /// @param y - y coordinaes
        /// @return std::vector<double> with elements being the mean of each vec
        std::vector<double> computeMean(vec x, vec y) {
            return {findMean(x), findMean(y)};
        }

        /// \brief helper function to find mean of a column vector 
        /// \param zi - column vec to find mean of 
        /// \return mean 
        double findMean(vec zi) {
            double sum_z = 0;
            for (size_t n = 0; n < zi.n_elem; n++) {
                sum_z += zi.at(n);
            }
            return sum_z / zi.n_elem;
        }

        /// @brief function to shift points to origin
        /// @param data_points - matrix of all x and y coor
        /// @param centroid - center of circle (from ComputeMean)
        /// @return matrix of x and y coordinates shifted
        mat shiftPoints(mat data_points, std::vector<double> centroid) {
            return arma::join_rows((data_points.col(0) - centroid.at(0)), (data_points.col(1) - centroid.at(1)));
        }

        /// @brief find zi, the distance of each point from the center
        /// @param data_shifted - the shifted matrix of all x and y points
        /// @return column vector of distances
        vec distToCentroid(mat data_shifted) {
            vec xi = data_shifted.col(0);
            vec yi = data_shifted.col(1);
            vec zi = xi; // temporarily assign values to zn
            for (size_t n = 0; n < xi.n_elem; n++) {
                zi.at(n) = (xi.at(n)*xi.at(n) + yi.at(n)*yi.at(n));
            }
            return zi;
        }

        /// @brief form data matrix from data points
        /// @param zi - all distances
        /// @param xi - all x coordinates
        /// @param yi - all y coordinates
        /// @return matrix of all Z
        mat formDataMat(vec zi, vec xi, vec yi) {
            return arma::join_rows(zi, xi, yi, vec(xi.n_elem, arma::fill::ones));
        }

        /// @brief form moment matrix
        /// @param Z - data matrix
        /// @return moment matrix
        mat formMoment(mat Z) {
            return (Z.t()) * Z * (1.0 / Z.n_rows);
        }
        
        /// @brief form constrint matrix for the "Hyperaccurate algebraic fit"
        /// @param z_bar - average distances
        /// @return 4x4 constraint matrix
        mat formConstraint(double z_bar) {
            mat H = mat(4, 4, arma::fill::eye);
            H.at(0, 0) = 8*z_bar;
            H.at(0, 3) = 2;
            H.at(3, 0) = 2;
            H.at(3, 3) = 0;
            return H;
        }

        /// @brief compute the column vec A to find circle equations
        /// @param Z - all data points matrix with distances
        /// @param H - constraint matrix
        /// @return column vector related to circle constants
        arma::vec computeA(mat Z, mat H) {
            arma::vec S;
            mat U, V;
            arma::svd(U, S, V, Z);
            // RCLCPP_ERROR_STREAM(get_logger(), "SVD: \n" << U << "\n" << S << "\n" << V);
            if (S(S.n_elem - 1) < 1e-12) {
                return V.row(3).t();
            }
            else {
                // RCLCPP_ERROR_STREAM(get_logger(), "inner: \n" << U << "\n" << S << "\n" << V);
                mat Y = V * arma::diagmat(S) * (V.t()); // results in a 4 x 3   
                mat Q = Y * (H.i()) * Y;
                arma::cx_vec eigval;
                arma::cx_mat eigvec;
                arma::eig_gen(eigval, eigvec, Q);
                double min_S = 1e12;
                double index = 0;
                for (size_t n = 0; n < eigval.n_elem; n++) {
                    if (eigval.at(n).real() < min_S && eigval.at(n).real() > 0) {
                        min_S = S.at(n);
                        index = n;
                    }
                }
                // RCLCPP_ERROR_STREAM(get_logger(), "smallest positive eigenvalue vector: \n" << eigvec(index));
                Q = arma::real(eigvec);
                return arma::solve(Y, Q.col(index));
            }
        }

        /// @brief find constnats to make the equation of a circle
        /// @param A - vector with coefficients from bigA (circle fitting algorithm)
        /// @return center (h,k) and R of a circle
        std::vector<double> circleEq(arma::vec A) {
            return {-A(1)/2/A(0), -A(2)/2/A(0), sqrt((A(1)*A(1)+A(2)*A(2)-4*A(0)*A(3))/(4*A(0)*A(0)))};
        }

        /// @brief calculate RMSE of points
        /// @param circle - std::vector with elements center (h,k) and R of a circle
        /// @param xcoor - x coordinate of points
        /// @param ycoor - y coordinate of points
        /// @return RMSE value
        double RMSE(std::vector<double> circle, vec xcoor, vec ycoor) {
            sum = 0;
            for (size_t i = 0; i < xcoor.n_elem; i++) {
                sum += pow(((xcoor(i) - circle.at(0))*(xcoor(i) - circle.at(0)) + (ycoor(i) - circle.at(1))*(ycoor(i) - circle.at(1)) - circle.at(2)*circle.at(2)), 2);
            }
            return sqrt(1.0 / xcoor.n_elem * sum);
        }

        /// @brief compute mean and stddev of points
        /// @param x_coor - x coordinate of points
        /// @param y_coor - y coordinate of points
        /// @return std::vector with mean and standard deviaton
        std::vector<double> computeStats(vec x_coor, vec y_coor) {
            angles = arma::zeros(x_coor.n_elem);
            // first, find distance for each point
            for (size_t loop = 0; loop < x_coor.n_elem; loop++) {
                angles(loop) = distance(0.0, 0.0, x_coor(loop), y_coor(loop));
            }
            // contains lowest to highest order of distance (i.e. max to min radius)
            indices = stable_sort_index(angles);
            // reset size of angles
            angles = arma::zeros(x_coor.n_elem - 2);
            for (size_t loop = 1; loop < (x_coor.n_elem - 1); loop++) {
                P1 = {(x_coor(indices(0)) - x_coor(indices(loop))), (y_coor(indices(0)) - y_coor(indices(loop)))};
                P2 = {(x_coor(indices(x_coor.n_elem - 1)) - x_coor(indices(loop))), (y_coor(indices(x_coor.n_elem - 1)) - y_coor(indices(loop)))};
                // RCLCPP_ERROR_STREAM(get_logger(), "P1: " << P1 << ", P2: " << P2);
                angles(loop-1) = turtlelib::normalize_angle(abs(turtlelib::angle(P1, P2)));
            }
            // RCLCPP_ERROR_STREAM(get_logger(), "angles: \n" << angles);
            return {arma::mean(angles), arma::stddev(angles)};
        }

        /// \brief helper function to check distance between points
        /// \param origin_x - current x coordinate
        /// \param origin_y - current y coordinate
        /// \param point_x - target x coordinate
        /// \param point_y - target y coordinate
        /// \return distance
        double distance(double origin_x, double origin_y, double point_x = 0.0, double point_y = 0.0) {
            return sqrt(pow((origin_x - point_x), 2.0) + pow((origin_y - point_y), 2.0));
        }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CircleFit>());
  rclcpp::shutdown();
  return 0;
}
