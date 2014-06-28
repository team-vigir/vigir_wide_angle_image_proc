#include <vigir_ocamlib_tools/ocamlib_camera_model.h>


namespace ocamlib_image_geometry{

OcamlibCameraModel::OcamlibCameraModel(const std::string& ocamlib_calibration_data_file)
{
  omni_camera_.reset(new vk::OmniCamera(ocamlib_calibration_data_file));
}

void OcamlibCameraModel::updateUndistortionLUT(int width, int height)
{
  mapx_persp_ = cv::Mat(height, width, CV_32FC1);
  mapy_persp_ = cv::Mat(height, width, CV_32FC1);

  this->create_perspective_undistortion_LUT(&mapx_persp_, &mapy_persp_, 4.0f);
}

void OcamlibCameraModel::rectifyImage(const cv::Mat& raw, cv::Mat& rectified,
                  int interpolation ) const
{
  cv::remap( raw, rectified, mapx_persp_, mapy_persp_, interpolation, cv::BORDER_CONSTANT, cv::Scalar(0,0, 0) );

}


void OcamlibCameraModel::create_perspective_undistortion_LUT( cv::Mat *mapx, cv::Mat *mapy, float sf) const
{
     int i, j;
     int width = mapx->cols; //New width
     int height = mapx->rows;//New height
     //float *data_mapx = mapx->data;
     //float *data_mapy = mapy->data;
     float Nxc = height/2.0f;
     float Nyc = width/2.0f;
     float Nz  = -width/sf;
     Eigen::Vector3d M;

     for (i=0; i<height; i++)
         for (j=0; j<width; j++)
         {
             M[0] = (i - Nxc);
             M[1] = (j - Nyc);
             M[2] = Nz;
             Eigen::Vector2d m (omni_camera_->world2cam(M));
             //*( data_mapx + i*width+j ) = (float) m[1];
             //*( data_mapy + i*width+j ) = (float) m[0];
             mapx->at<float>(i,j) = (float) m[1];
             mapy->at<float>(i,j) = (float) m[0];
         }
}


}
