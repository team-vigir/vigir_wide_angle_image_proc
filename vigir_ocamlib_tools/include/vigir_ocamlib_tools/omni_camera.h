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


/*
 * OcamProjector.h
 *
 *  Created on: Sep 22, 2010
 *      Author: laurent kneip
 */

// This has been copied and slightly modified from https://github.com/uzh-rpg/rpg_vikit mainly to keep the number of dependencies low

#ifndef OCAMPROJECTOR_H_
#define OCAMPROJECTOR_H_

#include <stdlib.h>
#include <string>
#include <Eigen/Eigen>
//#include <vikit/abstract_camera.h>
//#include <vikit/math_utils.h>

#define CMV_MAX_BUF 1024
#define MAX_POL_LENGTH 64

namespace vk {

using namespace std;
using namespace Eigen;

struct ocam_model
{
  double pol[MAX_POL_LENGTH];    	// the polynomial coefficients: pol[0] + x"pol[1] + x^2*pol[2] + ... + x^(N-1)*pol[N-1]
  int length_pol;                	// length of polynomial
  double invpol[MAX_POL_LENGTH]; 	// the coefficients of the inverse polynomial
  int length_invpol;             	// length of inverse polynomial
  double xc;				// row coordinate of the center
  double yc;         			// column coordinate of the center
  double c;				// affine parameter
  double d;				// affine parameter
  double e;				// affine parameter
  int width;				// image width
  int height;				// image height
};

class OmniCamera {//: public AbstractCamera {

private:
  struct ocam_model ocamModel;
  int width_; // TODO cannot be const because of omni-camera model
  int height_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double error_multiplier_;

  OmniCamera(){}
  OmniCamera(string calibFile);
  ~OmniCamera();

  //virtual
  Vector3d
  cam2world(const double& x, const double& y) const;

  //virtual
  Vector3d
  cam2world(const Vector2d& px) const;

  //virtual
  Vector2d
  world2cam(const Vector3d& xyz_c) const;

  //virtual Vector2d
  //world2cam(const Vector2d& uv) const;

  double
  computeErrorMultiplier();

  //virtual
  double errorMultiplier2() const
  {
    return sqrt(error_multiplier_)/2;
  }

  //virtual
  double errorMultiplier() const
  {
    return error_multiplier_;
  }

};

} // end namespace vk

#endif /* OCAMPROJECTOR_H_ */

