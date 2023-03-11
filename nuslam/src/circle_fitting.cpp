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
#include <armadillo>
#include <iostream>

using namespace std::chrono_literals;
using namespace arma;

class CircleFit : public rclcpp::Node {
    public:
    CircleFit()
    : Node("circle_fit"),
    rate(200.0)
    {
        declare_parameter("rate", rclcpp::ParameterValue(rate));
        get_parameter("rate", rate);

        cluster_sub = create_subscription<visualization_msgs::msg::MarkerArray>(
            "~/clusters", 100, std::bind(
                &CircleFit::cluster_callback, this, std::placeholders::_1)
            );
        timer = create_wall_timer(std::chrono::milliseconds(int(1.0 / rate * 1000)),
                                  std::bind(&CircleFit::timer_callback, this));
    }
    private:
        double rate;
        rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr cluster_sub;
        rclcpp::TimerBase::SharedPtr timer;
        mat clusters, data_points, bigZ, bigM, bigH, bigH_inv;
        arma::vec x_coor, y_coor, zi, bigA;
        std::vector<double> means, circle;
        double zbar;

        /// \brief Timer callback 
        void timer_callback() {
            ; // do stuff
        }

        /// \brief subscription callback for clusters
        /// \param clus - marker array from topic
        void cluster_callback(visualization_msgs::msg::MarkerArray clus) {
            for (size_t loop = 0; loop < clus.markers.size(); loop++) {
                if (clus.markers.at(loop).action == 0) {
                    x_coor.zeros(clus.markers.at(loop).points.size());
                    y_coor.zeros(clus.markers.at(loop).points.size());
                    for (size_t index = 0; index < clus.markers.at(loop).points.size(); index++) {
                        x_coor(index) = clus.markers.at(loop).points.at(index).x;
                        y_coor(index) = clus.markers.at(loop).points.at(index).y;
                    }
                    means = computeMean(x_coor, y_coor);
                    // RCLCPP_INFO(get_logger(), "xMean: %.7f, yMean: %.7f", means.at(0), means.at(1));
                    data_points = arma::join_rows(x_coor, y_coor);
                    // RCLCPP_ERROR_STREAM(get_logger(), "data: \n" << data_points);
                    x_coor = shiftPoints(data_points, means).col(0);
                    y_coor = shiftPoints(data_points, means).col(1);
                    zi = distToCentroid(shiftPoints(data_points, means));
                    zbar = findMean(zi);
                    bigZ = formDataMat(zi, x_coor, y_coor);
                    // RCLCPP_ERROR_STREAM(get_logger(), "capital Z: \n" << bigZ);
                    bigM = formMoment(bigZ);
                    // RCLCPP_ERROR_STREAM(get_logger(), "bigM: \n" << bigM);
                    bigH = formConstraint(zbar);
                    // RCLCPP_ERROR_STREAM(get_logger(), "bigH: \n" << bigH);
                    bigH_inv = bigH.i();
                    bigA = computeA(bigZ, bigH);
                    // RCLCPP_ERROR_STREAM(get_logger(), "bigA: \n" << bigA);
                    circle = circleEq(bigA);
                    // RCLCPP_ERROR_STREAM(get_logger(), "circle: \n" << (circle.at(0) + means.at(0)) << ", " << (circle.at(1) + means.at(1)) << ", " << sqrt(circle.at(2)));
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
            if (S(3) < 1e-12) {
                return V.col(3);
            }
            else {
                // RCLCPP_ERROR_STREAM(get_logger(), "SVD: \n" << U << "\n" << S << "\n" << V);
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

        std::vector<double> circleEq(arma::vec A) {
            return {-A(1)/2/A(0), -A(2)/2/A(0), ((A(1)*A(1)+A(2)*A(2)-4*A(0)*A(3))/(4*A(0)*A(0)))};
        }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CircleFit>());
  rclcpp::shutdown();
  return 0;
}
