#include <ros/ros.h>
#include <vigir_ocamlib_tools/ocamlib_camera_model_cv1.h>
#include <vigir_ocamlib_tools/math_tools.h>

int main(int argc, char **argv)
{
  /*
  //ros::init(argc, argv, "test_wide_angle");

  //ros::NodeHandle nh("~");

  Eigen::Vector3f direction(10.0f, 10.0f, 0.0f);

  Eigen::Matrix3f rot = ocamlib_image_geometry::getRotationFromDirection(direction, Eigen::Vector3f::UnitZ());

  std::cout << "\n" << rot << "\n";
  //ros::spin();


  Eigen::Vector3f direction2(10.0f, 0.0f, 10.0f);

  rot = ocamlib_image_geometry::getRotationFromDirection(direction2, Eigen::Vector3f::UnitZ());

  std::cout << "\n" << rot << "\n";

  Eigen::Vector3f direction3(0.0f, 1.0f, 0.0f);

  rot = ocamlib_image_geometry::getRotationFromDirection(direction3, Eigen::Vector3f::UnitZ());

  std::cout << "\n" << rot << "\n";


  ocamlib_image_geometry::OcamlibCameraModelCV1 model("/home/kohlbrecher/flor_repo/catkin_ws/src/vigir_wide_angle_image_proc/vigir_wide_angle_image_proc/calib/calib_results_l_sa.txt");

  //model.updateUndistortionLUT(800, 800, 4.0);

  //Eigen::Vector3d point_world(0.0, 10.0, -1.0);

  Eigen::Vector3d point_world(1.0, -1.0, 0.0);
  Eigen::Vector2d point_cam;
    std::cout << "\npoint_world:\n" <<  point_world << "\n";

    double test[2];
    test[0] = 0;
    test[1] = 0;
    double point_in_world[3];
    point_in_world[0] = point_world[0];
    point_in_world[1] = point_world[1];
    point_in_world[2] = point_world[2];

    double point_in_cam[3];
    point_in_cam[0] = point_in_world[1];
    point_in_cam[1] = -point_in_world[2];
    point_in_cam[2] = -point_in_world[0];

  //model.world2cam(point_world.data(), point_cam.data());
    model.world2cam(test, point_in_cam, model.getModel());

  //std::cout << "\n" <<  point_cam << "\n";

    std::cout << "\ntest:\n" << test[0] << " " << test[1] << "\n";
    */
  return 0;
}
