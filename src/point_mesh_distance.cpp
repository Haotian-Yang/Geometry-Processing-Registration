#include "point_mesh_distance.h"
#include "igl/per_face_normals.h"
#include "point_triangle_distance.h"

void point_mesh_distance(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & VY,
  const Eigen::MatrixXi & FY,
  Eigen::VectorXd & D,
  Eigen::MatrixXd & P,
  Eigen::MatrixXd & N)
{
  // init the matrix/vector
  P.resizeLike(X);
  N = Eigen::MatrixXd::Zero(X.rows(),X.cols());
  D.resize(X.rows());

  // Calculate all normals
  Eigen::MatrixXd N_all(FY.rows(), 3);
  igl::per_face_normals(VY, FY, N_all);

  for(int i = 0;i < X.rows();i++)
  {
      // keep track of the min target
      double d = std::numeric_limits<double>::max();
      double d_tmp;
      Eigen::RowVector3d closest_p, closest_p_tmp;
      int min_index = -1;

      for(int j = 0; j < FY.rows(); j++)
      {
          point_triangle_distance(X.row(i),
                                  VY.row(FY(j, 0)),
                                  VY.row(FY(j, 1)),
                                  VY.row(FY(j, 2)),
                                  d_tmp,
                                  closest_p_tmp);
          // find smaller face
          if(d_tmp < d)
          {
              d = d_tmp;
              min_index = j;
              closest_p = closest_p_tmp;
          }
      }
      P.row(i) = closest_p;
      D(i) = d;
      N.row(i) = N_all.row(min_index);
  }
}
