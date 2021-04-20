#include <Eigen/SVD>
#include <Eigen/Dense>
#include "closest_rotation.h"

void closest_rotation(
  const Eigen::Matrix3d & M,
  Eigen::Matrix3d & R)
{
  // Replace with your code
  Eigen::JacobiSVD<Eigen::MatrixXd> jacobiSvd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d U = jacobiSvd.matrixU();
  Eigen::Matrix3d V = jacobiSvd.matrixV();
  Eigen::Matrix3d Omega;
  double det = (U * V.transpose()).determinant();
  Omega << 1, 0, 0, 0, 1, 0, 0, 0, det;
  R = U * Omega * V.transpose();

}
