#include "point_to_plane_rigid_matching.h"
#include "closest_rotation.h"
#include <Eigen/Dense>

void point_to_plane_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  const Eigen::MatrixXd & N,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
  // Replace with your code
    int r = X.rows();

    // diag matrix
    Eigen::MatrixXd N_diag(r, 3 * r);
    N_diag << Eigen::MatrixXd(N.col(0).asDiagonal()),
            Eigen::MatrixXd(N.col(1).asDiagonal()),
            Eigen::MatrixXd(N.col(2).asDiagonal());
    // A
    Eigen::MatrixXd A;
    A = Eigen::MatrixXd::Zero(r * 3, 6);
    A << Eigen::VectorXd::Zero(r), X.col(2), -X.col(1), Eigen::VectorXd::Ones(r), Eigen::VectorXd::Zero(r), Eigen::VectorXd::Zero(r),
            -X.col(2), Eigen::VectorXd::Zero(r), X.col(0), Eigen::VectorXd::Zero(r), Eigen::VectorXd::Ones(r), Eigen::VectorXd::Zero(r),
            X.col(1), -X.col(0), Eigen::VectorXd::Zero(r), Eigen::VectorXd::Zero(r), Eigen::VectorXd::Zero(r), Eigen::VectorXd::Ones(r);
    A = N_diag * A;

    // b
    Eigen::VectorXd b(r * 3);
    b << X.col(0) - P.col(0),
         X.col(1) - P.col(1),
         X.col(2) - P.col(2);
    b = N_diag * b;

    // solve u*
    Eigen::VectorXd u;
    u = (A.transpose() * A).inverse() * (-A.transpose() * b);
    Eigen::Matrix3d M(3, 3);
    M <<  1, u(2), -u(1),
          -u(2), 1, u(0),
          u(1), -u(0), 1;
    closest_rotation(M,R);
    t = Eigen::Vector3d(u(3), u(4), u(5));
}
