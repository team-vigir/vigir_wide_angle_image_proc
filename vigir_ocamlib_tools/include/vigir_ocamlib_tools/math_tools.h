#ifndef MATH_TOOLS_H
#define MATH_TOOLS_H

#include <Eigen/Geometry>

namespace ocamlib_image_geometry{
 
  template<typename ConcreteScalar>
  Eigen::Matrix<ConcreteScalar, 3, 3> getRotationFromDirection(const Eigen::Matrix<ConcreteScalar, 3, 1>& direction, const Eigen::Matrix<ConcreteScalar, 3, 1>& up)
  {
    Eigen::Matrix<ConcreteScalar, 3, 3> ret;

    Eigen::Matrix<ConcreteScalar, 3, 1> direction_normalized(direction.normalized());

    Eigen::Matrix<ConcreteScalar, 3, 1> yaxis (up.cross(direction_normalized));
    yaxis.normalize();

    Eigen::Matrix<ConcreteScalar, 3, 1> zaxis (direction.cross(yaxis));
    zaxis.normalize();

    ret.col(0) = direction_normalized;
    ret.col(1) = yaxis;
    ret.col(2) = zaxis;

    return ret;
  }

  Eigen::Vector3f getProjection(const Eigen::Vector2f& image_coords, const Eigen::Vector2f& focal, const Eigen::Vector2f& optical_axis)
  {
    //Assume Z = 1 here
    Eigen::Vector3f vec;

    vec[0] = image_coords[0] - optical_axis[0] / focal[0];
    vec[1] = image_coords[1] - optical_axis[1] / focal[1];
    vec[2] = 1.0f;

    return vec;
  }
}


#endif
