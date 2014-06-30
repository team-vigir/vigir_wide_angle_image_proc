#include <ros/ros.h>
#include <vigir_ocamlib_tools/ocamlib_camera_model_cv1.h>
#include <vigir_ocamlib_tools/math_tools.h>

int main(int argc, char **argv)
{
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
  return 0;
}
