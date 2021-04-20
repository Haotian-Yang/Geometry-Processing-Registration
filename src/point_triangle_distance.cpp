#include "point_triangle_distance.h"

float clamp(float t)
{
    if(t>=0 && t<=1)
    {
        return t;
    }
    else if(t < 0)
    {
        return 0.0;
    }
    else
    {
        return 1.0;
    }
}

void point_triangle_distance(
  const Eigen::RowVector3d & x,
  const Eigen::RowVector3d & a,
  const Eigen::RowVector3d & b,
  const Eigen::RowVector3d & c,
  double & d,
  Eigen::RowVector3d & p)
{
    // Reference: Real-Time CollisionDetection, Christer Ericson Page 47.
    Eigen::RowVector3d v0 = b - a;
    Eigen::RowVector3d v1 = c - a;
    Eigen::RowVector3d v2 = x - a;
    float d00 = v0.dot(v0);
    float d01 = v0.dot(v1);
    float d11 = v1.dot(v1);
    float d20 = v2.dot(v0);
    float d21 = v2.dot(v1);
    float denom = (d00 * d11) - (d01 * d01);
    float v = (d11 * d20 - d01 * d21) / denom;
    float w = (d00 * d21 - d01 * d20) / denom;
    float u = 1.0f - v - w;

    // check if the projection is inside the triangle
    // The code is inspired by: https://stackoverflow.com/questions/14467296/barycentric-coordinate-clamping-on-3d-triangle
    if (u < 0)
    {
        float t = (p-b).dot(c-b)/((c-b).dot(c-b));
        t = clamp(t);
        u = 0.0f;
        v = 1.0f-t;
        w = t;
    }
    else if ( v < 0 )
    {
        float t = (p-c).dot(a-c)/(a-c).dot(a-c);
        t = clamp(t);
        u = t;
        v = 0.0f;
        w = 1.0f-t;
    }
    else if ( w < 0 )
    {
        float t = (p-a).dot(b-a)/(b-a).dot(b-a);
        t = clamp(t);
        u = 1.0f - t;
        v = t;
        w = 0.0f;
    }

    p = u * a + v * b + w * c;
    d = (p - x).norm();

  //  // Calculate the normal vector
  //Eigen::Vector3d ab = b - a;
  //Eigen::Vector3d ac = c - a;
  //Eigen::Vector3d ax = x - a;
  //Eigen::Vector3d n = ab.cross(ac);
  //n.normalize();
  //d = n.dot(ax);
  //Eigen:: Vector3d inv_n = -d*n;
  //p = ax - inv_n;


}
