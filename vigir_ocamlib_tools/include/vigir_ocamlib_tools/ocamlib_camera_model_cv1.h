#ifndef OCAMLIB_CAMERA_MODEL_H
#define OCAMLIB_CAMERA_MODEL_H

//#include <opencv2/core/core.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/calib3d/calib3d.hpp>
//#include <stdexcept>
#include <boost/shared_ptr.hpp>

#include <sensor_msgs/CameraInfo.h>

#include "omni_camera.h"
#include <opencv2/opencv.hpp>

struct ocam_model
{
  double pol[MAX_POL_LENGTH];    // the polynomial coefficients: pol[0] + x"pol[1] + x^2*pol[2] + ... + x^(N-1)*pol[N-1]
  int length_pol;                // length of polynomial
  double invpol[MAX_POL_LENGTH]; // the coefficients of the inverse polynomial
  int length_invpol;             // length of inverse polynomial
  double xc;         // row coordinate of the center
  double yc;         // column coordinate of the center
  double c;          // affine parameter
  double d;          // affine parameter
  double e;          // affine parameter
  int width;         // image width
  int height;        // image height
};



namespace ocamlib_image_geometry{


class OcamlibCameraModelCV1
{
    class RectifySettings
    {
    public:

        RectifySettings()
          : height(0), width(0), fc(0.0)
        {};

        bool settingsChanged(int height, int width, double fc, const Eigen::Vector3d& virtual_cam_direction, const Eigen::Vector3d& virtual_cam_up){
          return (height != this->height) ||
                 (width != this->width) ||
                 (fc != this->fc) ||
                 (virtual_cam_direction != virtual_cam_direction_) ||
                 (virtual_cam_up != virtual_cam_up_);
        }

        void updateSettings(int height, int width, double fc, const Eigen::Vector3d& virtual_cam_direction, const Eigen::Vector3d& virtual_cam_up)
        {
          this->height = height;
          this->width = width;
          this->fc = fc;
          virtual_cam_direction_ = virtual_cam_direction;
          virtual_cam_up_ = virtual_cam_up;
        }

    private:
        int height;
        int width;
        double fc;

        Eigen::Vector3d virtual_cam_direction_;
        Eigen::Vector3d virtual_cam_up_;
    };


public:

  OcamlibCameraModelCV1(const std::string& ocamlib_calibration_data_file);

  void rectifyImage(const cv::Mat& raw, cv::Mat& rectified,
                    int interpolation = CV_INTER_LINEAR) const;

  void updateUndistortionLUT(int width,
                             int height,
                             double fc,
                             const Eigen::Vector3d& direction = Eigen::Vector3d(1.0, 0.0, 0.0),
                             const Eigen::Vector3d& up = Eigen::Vector3d::UnitZ());

  void updateUndistortionLUT(cv::Mat *mapx,
                             cv::Mat *mapy,
                             float sf,
                             const Eigen::Matrix3d& transform_matrix);

  const Eigen::Matrix3d& getRotationMatrix() const { return rotation_eigen_; }

  void setCameraInfo(sensor_msgs::CameraInfo& cam_info) const { cam_info = *this->cam_info_; };

  void world2cam(const Eigen::Vector3d& world, Eigen::Vector2d& cam);
  void cam2world(const Eigen::Vector2d& cam, Eigen::Vector3d& world);



private:


  void create_perspective_undistortion_LUT( cv::Mat *mapx, cv::Mat *mapy, float sf) const;


  void cam2world(double point3D[3], double point2D[2], struct ocam_model *myocam_model );
  void world2cam(double point2D[2], double point3D[3], struct ocam_model *myocam_model );

  void cam2world(double *point3D, double *point2D);
  void world2cam(double *point2D, double *point3D);




  //void create_perspective_undistortion_LUT( cv::Mat *mapx, cv::Mat *mapy, struct ocam_model *ocam_model, float sf);

//<<<<<<< HEAD
  ocam_model * getModel() {return &o; };
//=======

//>>>>>>> master

protected:
  //boost::shared_ptr<vk::OmniCamera> omni_camera_;

  int get_ocam_model(struct ocam_model *myocam_model, const char *filename);

  CvMat* mapx_persp;
  CvMat* mapy_persp;

  cv::Mat mapx_persp_;
  cv::Mat mapy_persp_;

  ocam_model o;

  RectifySettings rectify_settings_;

  sensor_msgs::CameraInfoPtr cam_info_;

  // Holds the current virtual camera orientation (can be changed)
  Eigen::Matrix3d rotation_eigen_;

  Eigen::Matrix3d transform_matrix_;

  // Mapping to camera coords (stays fixed)
  Eigen::Matrix3d to_cam_;

};

}

#endif
