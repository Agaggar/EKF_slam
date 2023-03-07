#include <armadillo>
#include <iostream>

/*
given:
    * n data points (x_n, y_n)
*/

double findMean(arma::vec zi) {
    double sum_z = 0;
    for (size_t n = 0; n < zi.n_elem; n++) {
        sum_z += zi.at(n);
    }
    return sum_z / zi.n_elem;
}

std::vector<double> computeMean(arma::mat data_points) {
    arma::vec xn = data_points.col(0);
    arma::vec yn = data_points.col(1);
    return {findMean(xn), findMean(yn)};
}

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
    }
}

int main() {
    arma::mat M = {{1, 2, 3}, {4, 5, 6}};
    std::cout << M << std::endl;
    std::cout << M.col(0) << std::endl;
    return(0);
}