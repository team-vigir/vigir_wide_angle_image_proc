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


#ifndef OCAMLIB_CAMERA_MODEL_H
#define OCAMLIB_CAMERA_MODEL_H

//#include <opencv2/core/core.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/calib3d/calib3d.hpp>
//#include <stdexcept>
#include <boost/shared_ptr.hpp>

#include "omni_camera.h"
#include <opencv2/opencv.hpp>

namespace ocamlib_image_geometry{


class OcamlibCameraModel
{
public:

  OcamlibCameraModel(const std::string& ocamlib_calibration_data_file);

  void rectifyImage(const cv::Mat& raw, cv::Mat& rectified,
                    int interpolation = CV_INTER_LINEAR) const;

  void updateUndistortionLUT(int width, int height);
  void create_perspective_undistortion_LUT( cv::Mat *mapx, cv::Mat *mapy, float sf) const;
protected:
  boost::shared_ptr<vk::OmniCamera> omni_camera_;

  cv::Mat mapx_persp_;
  cv::Mat mapy_persp_;

};

}

#endif
