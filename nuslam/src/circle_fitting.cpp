/// \file
/// \brief circle fitting algorithm
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
        mat clusters;
        vec x_coor;
        vec y_coor;
        std::vector<double> means;

        /// \brief Timer callback 
        void timer_callback() {
            ; // do stuff
        }

        /// \brief subscription callback for clusters
        /// \param clus - marker array from topic
        void cluster_callback(visualization_msgs::msg::MarkerArray clus) {
            // received_clusters.resize(clus.markers.size());
            for (size_t loop = 0; loop < clus.markers.size(); loop++) {
                // received_clusters.at(loop) = {};
                if (clus.markers.at(loop).action == 0) {
                    x_coor.zeros(clus.markers.at(loop).points.size());
                    y_coor.zeros(clus.markers.at(loop).points.size());
                    for (size_t index = 0; index < clus.markers.at(loop).points.size(); index++) {
                        x_coor(index) = clus.markers.at(loop).points.at(index).x;
                        y_coor(index) = clus.markers.at(loop).points.at(index).x;
                        // received_clusters.at(loop).push_back(clus.markers.at(loop).points.at(index).x);
                        // received_clusters.at(loop).push_back(clus.markers.at(loop).points.at(index).y);
                    }
                    means = computeMean(x_coor, y_coor);
                    RCLCPP_ERROR_STREAM(get_logger(), "x: , y: \n" << arma::join_rows(x_coor, y_coor));
                    RCLCPP_INFO(get_logger(), "xMean: %.4f, yMean: %.4f", means.at(0), means.at(1));
                }
            }
        }

        /// @brief find mean of two column vectors
        /// @param x - x coordinates
        /// @param y - y coordinaes
        /// @return std::vector<double> with elements being the mean of each vec
        std::vector<double> computeMean(arma::vec x, arma::vec y) {
            return {findMean(x), findMean(y)};
        }

        /// \brief helper function to find mean of a column vector 
        /// \param zi - column vec to find mean of 
        /// \return mean 
        double findMean(arma::vec zi) {
            double sum_z = 0;
            for (size_t n = 0; n < zi.n_elem; n++) {
                sum_z += zi.at(n);
            }
            return sum_z / zi.n_elem;
        }

        /// @brief function to shift points to origin
        /// @param data_points 
        /// @param centroid 
        /// @return 
        arma::mat shiftPoints(arma::mat data_points, std::vector<double> centroid) {
            return arma::join_cols((data_points.col(0) - centroid.at(0)), (data_points.col(1) - centroid.at(1)));
        }

        arma::vec distToCentroid(arma::mat data_shifted) {
            arma::vec xi = data_shifted.col(0);
            arma::vec yi = data_shifted.col(1);
            arma::vec zi = xi; // temporarily assign values to zn
            for (size_t n = 0; n < xi.n_elem; n++) {
                zi.at(n) = (xi.at(n)*xi.at(n) + yi.at(n)*yi.at(n));
            }
            return zi;
        }

        arma::mat formDataMat(arma::vec zi, arma::vec xi, arma::vec yi) {
            return arma::join_cols(zi, xi, yi, arma::vec(xi.n_elem, arma::fill::ones));
        }

        arma::mat formMoment(arma::mat Z) {
            return (1.0 / Z.n_rows) * (Z.t()) * Z;
        }

        arma::mat formConstraint(double z_bar) {
            arma::mat H = arma::mat(4, 4, arma::fill::eye);
            H.at(0, 0) = 8*z_bar;
            H.at(0, 3) = 2;
            H.at(3, 0) = 2;
            H.at(3, 3) = 0;
            return H;
        }

        arma::mat computeInv(arma::mat H) {
            return arma::inv(H);
        }

        arma::vec computeA(arma::mat Z, arma::mat H) {
            arma::vec S;
            arma::mat U, V;
            arma::svd(U, S, V, Z);
            double min_S = 1e12;
            for (size_t n = 0; n < S.n_elem; n++) {
                if (S.at(n) < min_S) {
                    min_S = S.at(n);
                }
            }
            if (min_S < 1e-12) {
                return V.col(3);
            }
            else {
                arma::mat Y = V * arma::diagmat(S) * (V.t());
                arma::mat Q = Y * (computeInv(H)) * Y;
                arma::cx_vec eigval;
                arma::cx_mat eigvec;
                arma::eig_gen(eigval, eigvec, Q);
                min_S = 1e12;
                double index = 0;
                for (size_t n = 0; n < eigval.n_elem; n++) {
                    if (eigval.at(n).real() < min_S && eigval.at(n).real() > 0) {
                        min_S = S.at(n);
                        index = n;
                    }
                }
                // find eigenvector associated with said eigenvalue (A*), then compute A 
                // return eigvec.at(index, index) * (arma::inv(Y))
                // temp return function
                return V.col(3);
            }
        }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CircleFit>());
  rclcpp::shutdown();
  return 0;
}
