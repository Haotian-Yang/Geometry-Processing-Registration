#include "random_points_on_mesh.h"
#include <random>
#include "igl/cumsum.h"
#include "igl/doublearea.h"

void random_points_on_mesh(
  const int n,
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  Eigen::MatrixXd & X)
{
  // REPLACE WITH YOUR CODE:
  X.resize(n,3);

  Eigen::MatrixXd F_area, F_cumsum;
  // Calculate the per-traiangle areas
  igl::doublearea(V, F, F_area);
  // Calculate the cumulative sum
  igl::cumsum(F_area/2.0, 1, F_cumsum);
  F_cumsum /= F_cumsum(F_cumsum.rows()-1); // devide by the total area

  std::default_random_engine generator;
  std::uniform_real_distribution<double> distribution(0.0,1.0);
  for(int i = 0;i<X.rows();i++)
  {
      double a1 = distribution(generator);
      double a2 = distribution(generator);

      // reflect back the select point x back into the original triangle
      if(a1 + a2 > 1)
      {
          a1 = 1 - a1;
          a2 = 1 - a2;
      }

      // draw the index of triangle by find the first entry in F_cumsum
      // whose value > a2.
      int T_index;
      for (int i=0; i<F_cumsum.rows(); i++)
      {
          if(F_cumsum(i) > a2)
          {
              T_index = i;
              break;
          }
      }

      int v1_index = F(T_index, 0);
      int v2_index = F(T_index, 1);
      int v3_index = F(T_index, 2);
      Eigen::VectorXd v1 = V.row(v1_index);
      Eigen::VectorXd v2 = V.row(v2_index);
      Eigen::VectorXd v3 = V.row(v3_index);

      // x = v1 + a1(v2 - v1) + a2(v3 - v1)
      X.row(i) = v1 + (a1*(v2 - v1)) + (a2*(v3 - v1));
  }
}

