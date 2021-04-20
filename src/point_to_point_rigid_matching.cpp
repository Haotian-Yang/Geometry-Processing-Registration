#include "point_to_point_rigid_matching.h"
#include "closest_rotation.h"

void point_to_point_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
  // Replace with your code
  // Calculate mean
  Eigen::RowVector3d X_mean = X.colwise().mean();
  Eigen::RowVector3d P_mean = P.colwise().mean();

  // Minus mean
  Eigen::MatrixXd X_hat = X.rowwise() - X_mean;
  Eigen::MatrixXd P_hat = P.rowwise() - P_mean;

  // Solve R, t
  Eigen::MatrixXd M = (P_hat.transpose()*X_hat).transpose();
  closest_rotation(M, R);
  t = (P_mean.transpose() - (R*(X_mean.transpose()))).transpose();
}

