#include <vigir_ocamlib_tools/ocamlib_camera_model_cv1.h>

#include <opencv/cv.h>
#include <Eigen/Geometry>
#include <vigir_ocamlib_tools/math_tools.h>

namespace ocamlib_image_geometry{

OcamlibCameraModelCV1::OcamlibCameraModelCV1(const std::string& ocamlib_calibration_data_file)
  : rotation_eigen_(Eigen::Matrix3d::Identity())
{
  to_cam_ << 0, 0, 1,    0, 1 ,0,    1,  0, 0;

  //omni_camera_.reset(new vk::OmniCamera(ocamlib_calibration_data_file));

  get_ocam_model(&o, ocamlib_calibration_data_file.c_str());

  mapx_persp = 0;
  mapy_persp = 0;

  cam_info_.reset(new sensor_msgs::CameraInfo());
}

void OcamlibCameraModelCV1::updateUndistortionLUT(int height,
                                                  int width,
                                                  double fc,
                                                  const Eigen::Vector3d& direction,
                                                  const Eigen::Vector3d& up)
{
  if (rectify_settings_.settingsChanged(height, width, fc)){

    mapx_persp_ = cv::Mat(height, width, CV_32FC1);
    mapy_persp_ = cv::Mat(height, width, CV_32FC1);


    //create_perspective_undistortion_LUT( &mapx_persp_, &mapy_persp_, &o, fc );
    this->updateUndistortionLUT(&mapx_persp_, &mapy_persp_, fc, direction, up);
    rectify_settings_.updateSettings(height, width, fc);

    // @TODO: Find out if optical center estimate or ideal optical center should be used here

    cam_info_->height = height;
    cam_info_->width  = width;
    cam_info_->P[0]  = width / fc;
    //cam_info_->P[2]  = width/2.0;
    cam_info_->P[2]  = o.xc;
    cam_info_->P[5]  = width / fc;
    //cam_info_->P[6]  = height/2.0;
    cam_info_->P[6]  = o.yc;
    cam_info_->P[10] = 1.0;
  }
}



void OcamlibCameraModelCV1::rectifyImage(const cv::Mat& raw, cv::Mat& rectified,
                  int interpolation ) const
{
  cv::remap( raw, rectified, mapx_persp_, mapy_persp_, interpolation, cv::BORDER_CONSTANT, cv::Scalar(0,0, 0) );

}


void OcamlibCameraModelCV1::cam2world(double point3D[3], double point2D[2], struct ocam_model *myocam_model)
{
 double *pol    = myocam_model->pol;
 double xc      = (myocam_model->xc);
 double yc      = (myocam_model->yc);
 double c       = (myocam_model->c);
 double d       = (myocam_model->d);
 double e       = (myocam_model->e);
 int length_pol = (myocam_model->length_pol);
 double invdet  = 1/(c-d*e); // 1/det(A), where A = [c,d;e,1] as in the Matlab file

 double xp = invdet*(    (point2D[0] - xc) - d*(point2D[1] - yc) );
 double yp = invdet*( -e*(point2D[0] - xc) + c*(point2D[1] - yc) );

 double r   = sqrt(  xp*xp + yp*yp ); //distance [pixels] of  the point from the image center
 double zp  = pol[0];
 double r_i = 1;
 int i;

 for (i = 1; i < length_pol; i++)
 {
   r_i *= r;
   zp  += r_i*pol[i];
 }

 //normalize to unit norm
 double invnorm = 1/sqrt( xp*xp + yp*yp + zp*zp );

 point3D[0] = invnorm*xp;
 point3D[1] = invnorm*yp;
 point3D[2] = invnorm*zp;
}

//------------------------------------------------------------------------------
void OcamlibCameraModelCV1::world2cam(double point2D[2], double point3D[3], struct ocam_model *myocam_model)
{
 double *invpol     = myocam_model->invpol;
 double xc          = (myocam_model->xc);
 double yc          = (myocam_model->yc);
 double c           = (myocam_model->c);
 double d           = (myocam_model->d);
 double e           = (myocam_model->e);
 int    width       = (myocam_model->width);
 int    height      = (myocam_model->height);
 int length_invpol  = (myocam_model->length_invpol);
 double norm        = sqrt(point3D[0]*point3D[0] + point3D[1]*point3D[1]);
 double theta       = atan(point3D[2]/norm);
 double t, t_i;
 double rho, x, y;
 double invnorm;
 int i;
 //std::cout << "bla2\n";
 //std::cout << "\n" << point3D[0] << "   " << point3D[1] << "   " << point3D[2] << "\n";

  if (norm != 0)
  {
    invnorm = 1/norm;
    t  = theta;
    rho = invpol[0];
    t_i = 1;

    for (i = 1; i < length_invpol; i++)
    {
      t_i *= t;
      rho += t_i*invpol[i];
    }

    x = point3D[0]*invnorm*rho;
    y = point3D[1]*invnorm*rho;

    point2D[0] = x*c + y*d + xc;
    point2D[1] = x*e + y   + yc;

    //std::cout << "\n first " << point2D[0] << "   " << point2D[1] << "\n";
  }
  else
  {
    point2D[0] = xc;
    point2D[1] = yc;
    //std::cout << "\n second " << point2D[0] << "   " << point2D[1] << "\n";
  }
}

void OcamlibCameraModelCV1::cam2world(double point3D[3], double point2D[2])
{
  cam2world(point3D, point2D, &o);
}

void OcamlibCameraModelCV1::world2cam(double* point2D, double* point3D)
{
  std::cout << "bla\n";
  world2cam(point2D, point3D, &o);
}

void OcamlibCameraModelCV1::create_perspective_undistortion_LUT( cv::Mat *mapx, cv::Mat *mapy, struct ocam_model *ocam_model, float sf)
{
     int i, j;
     int width = mapx->cols; //New width
     int height = mapx->rows;//New height
     float Nxc = height/2.0;
     float Nyc = width/2.0;
     float Nz  = -width/sf;
     double M[3];
     double m[2];

     for (i=0; i<height; i++)
         for (j=0; j<width; j++)
         {
             M[0] = (i - Nxc);
             M[1] = (j - Nyc);
             M[2] = Nz;
             world2cam(m, M, ocam_model);

             mapx->at<float>(i,j) = (float) m[1];
             mapy->at<float>(i,j) = (float) m[0];
         }
}

void OcamlibCameraModelCV1::updateUndistortionLUT(cv::Mat *mapx,
                                                  cv::Mat *mapy,
                                                  float sf,
                                                  const Eigen::Vector3d& direction,
                                                  const Eigen::Vector3d& up)
{
  int width = mapx->cols; //New width
  int height = mapx->rows;//New height

  rotation_eigen_ = (ocamlib_image_geometry::getRotationFromDirection(direction, up));

  double Nxc = height/2.0;
  double Nyc = width/2.0;
  double Nz  = -width/sf;

  //double M[3];
  double m[2];

  Eigen::Matrix3d transform_matrix (to_cam_.transpose() * rotation_eigen_  * to_cam_);
  //Eigen::Matrix3f transform_matrix (to_cam * rotation_eigen );

  //std::cout << "\nTransform Matrix\n" << transform_matrix << "\n";

  Eigen::Vector3d world_vec_pre_rotate;
  for (int i=0; i<height; i++){
      for (int j=0; j<width; j++)
      {
          /*
          world_vec_pre_rotate[0] = (i - Nxc);
          world_vec_pre_rotate[1] = (j - Nyc);
          world_vec_pre_rotate[2] = Nz;
          */


        Eigen::Vector3d world_vec_rotated (transform_matrix * Eigen::Vector3d(i - Nxc,
                                                                              j - Nyc,
                                                                              Nz));

          /*
          M[0] = world_vec_rotated[0];
          M[1] = world_vec_rotated[1];
          M[2] = world_vec_rotated[2];
          */

          this->world2cam(m, world_vec_rotated.data(), &o);

          mapx->at<float>(i,j) = (float) m[1];
          mapy->at<float>(i,j) = (float) m[0];
      }
  }




  //rotation_eigen = (Eigen::AngleAxisf(0.25*M_PI, Eigen::Vector3f::UnitX()) *
  //                  Eigen::AngleAxisf(0.25*M_PI, Eigen::Vector3f::UnitY())).matrix();

  //std::vector<std::pair<


  //cv::projectPoints()



}

int OcamlibCameraModelCV1::get_ocam_model(struct ocam_model *myocam_model, const char *filename)
{
 double *pol        = myocam_model->pol;
 double *invpol     = myocam_model->invpol;
 double *xc         = &(myocam_model->xc);
 double *yc         = &(myocam_model->yc);
 double *c          = &(myocam_model->c);
 double *d          = &(myocam_model->d);
 double *e          = &(myocam_model->e);
 int    *width      = &(myocam_model->width);
 int    *height     = &(myocam_model->height);
 int *length_pol    = &(myocam_model->length_pol);
 int *length_invpol = &(myocam_model->length_invpol);
 FILE *f;
 char buf[CMV_MAX_BUF];
 int i;

 //Open file
 if(!(f=fopen(filename,"r")))
 {
   printf("File %s cannot be opened\n", filename);
   return -1;
 }

 //Read polynomial coefficients
 fgets(buf,CMV_MAX_BUF,f);
 fscanf(f,"\n");
 fscanf(f,"%d", length_pol);
 for (i = 0; i < *length_pol; i++)
 {
     fscanf(f," %lf",&pol[i]);
 }

 //Read inverse polynomial coefficients
 fscanf(f,"\n");
 fgets(buf,CMV_MAX_BUF,f);
 fscanf(f,"\n");
 fscanf(f,"%d", length_invpol);
 for (i = 0; i < *length_invpol; i++)
 {
     fscanf(f," %lf",&invpol[i]);
 }

 //Read center coordinates
 fscanf(f,"\n");
 fgets(buf,CMV_MAX_BUF,f);
 fscanf(f,"\n");
 fscanf(f,"%lf %lf\n", xc, yc);

 //Read affine coefficients
 fgets(buf,CMV_MAX_BUF,f);
 fscanf(f,"\n");
 fscanf(f,"%lf %lf %lf\n", c,d,e);

 //Read image size
 fgets(buf,CMV_MAX_BUF,f);
 fscanf(f,"\n");
 fscanf(f,"%d %d", height, width);

 fclose(f);
 return 0;
}


}
