#include <Eigen/Core>
#include <Eigen/Dense>

#include "ndcurves/piecewise_curve.h"
#include "ndcurves/polynomial.h"

using Matrix3d = Eigen::Matrix<double, 3, 3>;
using Matrix2d = Eigen::Matrix<double, 2, 2>;
using Vector3d = Eigen::Matrix<double, 3, 1>;
using Vector2d = Eigen::Matrix<double, 2, 1>;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::VectorXd VectorXd;

typedef Eigen::Vector3d point3;
typedef std::vector<point3, Eigen::aligned_allocator<point3>> t_point3;

typedef ndcurves::polynomial<double, double, true, point3> Polynomial;
typedef ndcurves::piecewise_curve<double, double, true, point3, point3,
                                  Polynomial>
    PieceWise;