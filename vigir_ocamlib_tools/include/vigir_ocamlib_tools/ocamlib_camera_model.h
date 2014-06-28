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
