/*
 *  Copyright (C) 2014, FG SIM, TU Darmstadt (Team ViGIR)
 *  Stefan Kohlbrecher <kohlbrecher@sim.tu-darmstadt.de>
 *
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


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
