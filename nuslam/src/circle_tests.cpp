/// turn this into a library, use in circle fitting node
#include <armadillo>
#include <iostream>

using namespace arma;

/// \brief helper function to find mean of a column vector
/// \param zi - column vec to find mean of
/// \return mean
double findMean(vec zi)
{
  double sum_z = 0;
  for (size_t n = 0; n < zi.n_elem; n++) {
    sum_z += zi.at(n);
  }
  return sum_z / zi.n_elem;
}

/// @brief find mean of two column vectors
/// @param x - x coordinates
/// @param y - y coordinaes
/// @return std::vector<double> with elements being the mean of each vec
std::vector<double> computeMean(vec x, vec y)
{
  return {findMean(x), findMean(y)};
}

/// @brief function to shift points to origin
/// @param data_points - matrix of all x and y coor
/// @param centroid - center of circle (from ComputeMean)
/// @return matrix of x and y coordinates shifted
mat shiftPoints(mat data_points, std::vector<double> centroid)
{
  return arma::join_rows(
    (data_points.col(0) - centroid.at(0)), (data_points.col(1) - centroid.at(
      1)));
}

/// @brief find zi, the distance of each point from the center
/// @param data_shifted - the shifted matrix of all x and y points
/// @return column vector of distances
vec distToCentroid(mat data_shifted)
{
  vec xi = data_shifted.col(0);
  vec yi = data_shifted.col(1);
  vec zi = xi;   // temporarily assign values to zn
  for (size_t n = 0; n < xi.n_elem; n++) {
    zi.at(n) = (xi.at(n) * xi.at(n) + yi.at(n) * yi.at(n));
  }
  return zi;
}

/// @brief form data matrix from data points
/// @param zi - all distances
/// @param xi - all x coordinates
/// @param yi - all y coordinates
/// @return matrix of all Z
mat formDataMat(vec zi, vec xi, vec yi)
{
  return arma::join_rows(zi, xi, yi, vec(xi.n_elem, arma::fill::ones));
}

/// @brief form moment matrix
/// @param Z - data matrix
/// @return moment matrix
mat formMoment(mat Z)
{
  return (Z.t()) * Z * (1.0 / Z.n_rows);
}

/// @brief form constrint matrix for the "Hyperaccurate algebraic fit"
/// @param z_bar - average distances
/// @return 4x4 constraint matrix
mat formConstraint(double z_bar)
{
  mat H = mat(4, 4, arma::fill::eye);
  H.at(0, 0) = 8 * z_bar;
  H.at(0, 3) = 2;
  H.at(3, 0) = 2;
  H.at(3, 3) = 0;
  return H;
}

/// @brief compute the column vec A to find circle equations
/// @param Z - all data points matrix with distances
/// @param H - constraint matrix
/// @return column vector related to circle constants
arma::vec computeA(mat Z, mat H)
{
  arma::vec S;
  mat U, V;
  arma::svd(U, S, V, Z, "std");
  if (S(3) < 1e-12) {
    return V.col(3);
  } else {
    // RCLCPP_ERROR_STREAM(get_logger(), "SVD: \n" << U << "\n" << S << "\n" << V);
    mat Y = V * arma::diagmat(S) * (V.t());     // results in a 4 x 3
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
    // RCLCPP_ERROR_STREAM(get_logger(), "smalles positive eigenvalue vector: \n" << eigvec(index));
    Q = arma::real(eigvec);
    return arma::solve(Y, Q.col(index));
  }
}

std::vector<double> circleEq(arma::vec A)
{
  return {-A(1) / 2 / A(0), -A(2) / 2 / A(0),
    ((A(1) * A(1) + A(2) * A(2) - 4 * A(0) * A(3)) / (4 * A(0) * A(0)))};
}

int main()
{
  arma::mat test1 = {{1, 7}, {2, 6}, {5, 8}, {7, 7}, {9, 5}, {3, 7}};
  std::vector<double> means = computeMean(test1.col(0), test1.col(1));
  vec x_coor = shiftPoints(test1, means).col(0);
  vec y_coor = shiftPoints(test1, means).col(1);
  vec zi = distToCentroid(shiftPoints(test1, means));
  double zbar = findMean(zi);
  mat bigZ = formDataMat(zi, x_coor, y_coor);
  mat bigH = formConstraint(zbar);
  vec bigA = computeA(bigZ, bigH);
  std::vector<double> coeff = circleEq(bigA);
  std::cout << (coeff.at(0) + means.at(0)) << ", " << (coeff.at(1) + means.at(1)) << ", " << sqrt(
    coeff.at(
      2)) << std::endl;
  return 0;
}
