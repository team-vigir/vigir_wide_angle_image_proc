/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#include <boost/version.hpp>
#if ((BOOST_VERSION / 100) % 1000) >= 53
#include <boost/thread/lock_guard.hpp>
#endif

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <vigir_wide_angle_image_proc/RectifyConfig.h>

#include <vigir_ocamlib_tools/ocamlib_camera_model_cv1.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <sensor_msgs/JointState.h>

namespace wide_angle_image_proc {

class RectifyNodelet : public nodelet::Nodelet
{
  // ROS communication
  boost::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::CameraSubscriber sub_camera_;

  int queue_size_;

  std::string rectified_frame_id_;
  std::string parent_frame_id_;

  std::string yaw_joint_name_;
  std::string pitch_joint_name_;
  
  boost::mutex connect_mutex_;
  //image_transport::Publisher pub_rect_;
  image_transport::CameraPublisher pub_rect_camera_;

  // Dynamic reconfigure
  boost::recursive_mutex config_mutex_;
  typedef vigir_wide_angle_image_proc::RectifyConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;
  Config config_;

  // Processing state (note: only safe because we're using single-threaded NodeHandle!)
  //image_geometry::PinholeCameraModel model_;
  boost::shared_ptr<ocamlib_image_geometry::OcamlibCameraModelCV1> model_;


  //Optional
  boost::shared_ptr<tf::TransformBroadcaster> tfb_;
  boost::shared_ptr<tf::TransformListener> tfl_;
  boost::shared_ptr<ros::Publisher> joint_state_pub_;

  virtual void onInit();

  void connectCb();

  void imageCb(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg);

  void configCb(Config &config, uint32_t level);
};

void RectifyNodelet::onInit()
{
  ros::NodeHandle &nh         = getNodeHandle();
  ros::NodeHandle &private_nh = getPrivateNodeHandle();

  //ros::NodeHandle rect_nh (getPrivateNodeHandle().getNamespace()); //+ "/rect_camera");
  it_.reset(new image_transport::ImageTransport(nh));

  // Read parameters
  private_nh.param("queue_size", queue_size_, 5);
  std::string calibration_text_file;
  private_nh.param("calibration_text_file", calibration_text_file, std::string("N/A"));
  NODELET_INFO("Loaded ocamlib calibration file %s", calibration_text_file.c_str());

  private_nh.param("rectified_frame_id", rectified_frame_id_, std::string(""));
  private_nh.param("parent_frame_id", parent_frame_id_, std::string(""));

  if(rectified_frame_id_.empty()){
    NODELET_INFO("No rectified frame_id specified, using subscribed image frame_id");
  }else{
    NODELET_INFO("Using frame_id: %s for rectified images", rectified_frame_id_.c_str());
  }


  bool use_tfb = false;
  private_nh.param("use_tf_broadcaster", use_tfb, false);

  bool use_tfl = false;
  private_nh.param("use_tf_listener", use_tfl, false);

  if (use_tfb){
    this->tfb_.reset(new tf::TransformBroadcaster());
  }

  if (use_tfl){
    this->tfl_.reset(new tf::TransformListener());
  }

  bool pub_joint_states = false;
  private_nh.param("pub_joint_states", use_tfl, false);

  if (pub_joint_states){
    private_nh.param("yaw_joint_name", yaw_joint_name_, std::string(""));
    private_nh.param("pitch_joint_name", pitch_joint_name_, std::string(""));

    if (!yaw_joint_name_.empty() && !pitch_joint_name_.empty()){
      this->joint_state_pub_.reset(new ros::Publisher());
      nh.advertise<sensor_msgs::JointState>("joint_states", 3, false);
    }else{
      NODELET_ERROR("No joint names set for yaw and pitch angle, not publishing joint_state");
    }

  }

  model_.reset(new ocamlib_image_geometry::OcamlibCameraModelCV1(calibration_text_file));

  // Set up dynamic reconfigure
  reconfigure_server_.reset(new ReconfigureServer(config_mutex_, private_nh));
  ReconfigureServer::CallbackType f = boost::bind(&RectifyNodelet::configCb, this, _1, _2);
  reconfigure_server_->setCallback(f);

  // Monitor whether anyone is subscribed to the output
  image_transport::SubscriberStatusCallback connect_cb = boost::bind(&RectifyNodelet::connectCb, this);
  // Make sure we don't enter connectCb() between advertising and assigning to pub_rect_
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  //pub_rect_  = it_->advertise("image_rect",  1, connect_cb, connect_cb);
  pub_rect_camera_ = it_->advertiseCamera("image_rect_ns", 1, connect_cb, connect_cb);
}

// Handles (un)subscribing when clients (un)subscribe
void RectifyNodelet::connectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if (pub_rect_camera_.getNumSubscribers() == 0){
    NODELET_INFO("Disconnecting.");
    sub_camera_.shutdown();
  }else if (!sub_camera_){
    NODELET_INFO("Connecting.");
    image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
    sub_camera_ = it_->subscribeCamera("image_mono", queue_size_, &RectifyNodelet::imageCb, this, hints);
  }
}

void RectifyNodelet::imageCb(const sensor_msgs::ImageConstPtr& image_msg,
                             const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  // Verify camera is actually calibrated
  if (!info_msg->K[0] == 0.0) {
    NODELET_ERROR_THROTTLE(30, "Rectified topic '%s' requested but camera publishing '%s' "
                           "has non-zero CameraInfo.", pub_rect_camera_.getTopic().c_str(),
                           sub_camera_.getInfoTopic().c_str());
    return;
  }

  if (rectified_frame_id_ == ""){
    rectified_frame_id_ = image_msg->header.frame_id;
  }
  
  // Create cv::Mat views onto both buffers
  const cv::Mat image = cv_bridge::toCvShare(image_msg)->image;
  cv::Mat rect;

  // Rectify and publish
  int interpolation;
  {
    boost::lock_guard<boost::recursive_mutex> lock(config_mutex_);
    //interpolation = config_.interpolation;
  }

  Eigen::Vector3d virtual_cam_direction (Eigen::Vector3d::UnitX());

  if (config_.use_reconfigure_viewpoint){
    virtual_cam_direction = (Eigen::AngleAxisd((config_.yaw_deg * M_PI)/180.0, Eigen::Vector3d::UnitZ()) *
                             Eigen::AngleAxisd((config_.pitch_deg * M_PI)/180.0, Eigen::Vector3d::UnitY()) *
                             Eigen::Vector3d::UnitX());
  }else if (tfl_){
    try{
      if (!config_.track_frame.empty()){
        tf::StampedTransform transform;
        tfl_->lookupTransform( this->parent_frame_id_, std::string(config_.track_frame), ros::Time(0),  transform);

        tf::Vector3 target_dir = transform * tf::Vector3(config_.track_frame_x,
                                                         config_.track_frame_y,
                                                         config_.track_frame_z);

        tf::vectorTFToEigen(target_dir, virtual_cam_direction);
      }
    }catch(tf::TransformException e){
      NODELET_ERROR("Transfom error: %s", e.what());
    }
  }



  Eigen::Vector3d virtual_cam_up_vector (Eigen::Vector3d::UnitZ());

  // This only performs actual processing when settings changed
  model_->updateUndistortionLUT(image_msg->height, image_msg->width, config_.focal_length, virtual_cam_direction, virtual_cam_up_vector);

  model_->rectifyImage(image, rect, config_.interpolation);

  // Allocate new rectified image message
  sensor_msgs::ImagePtr rect_msg = cv_bridge::CvImage(image_msg->header, image_msg->encoding, rect).toImageMsg();
  sensor_msgs::CameraInfoPtr rect_info (new sensor_msgs::CameraInfo());

  model_->setCameraInfo(*rect_info);
  rect_info->header = info_msg->header;

  rect_info->header.frame_id = rectified_frame_id_ + "_optical_frame";
  rect_msg->header.frame_id = rectified_frame_id_ + "_optical_frame";

  pub_rect_camera_.publish(rect_msg, rect_info);

  // Publish cam frames to tf if requested
  if (tfb_){

    // Transform from parent to (non optical) camera frame
    const Eigen::Matrix3d& rotation_eigen = model_->getRotationMatrix();


    tf::Quaternion tf_quat;
    tf::quaternionEigenToTF(Eigen::Quaterniond (rotation_eigen), tf_quat);

    tf::StampedTransform trans;
    trans.child_frame_id_ = rectified_frame_id_;
    trans.frame_id_ = parent_frame_id_;

    trans.setIdentity();
    trans.setRotation(tf_quat);

    // Publishing two different tf transforms with the image timestamp inbetween to make sure that
    // we don´t get interpolation artifacts
    std::vector<tf::StampedTransform> tf_vec;
    trans.stamp_ = image_msg->header.stamp - ros::Duration(0.001);
    tf_vec.push_back(trans);

    trans.stamp_ = image_msg->header.stamp + ros::Duration(0.001);
    tf_vec.push_back(trans);



    // Transform from camera to optical frame
    tf::Quaternion to_optical;
    to_optical.setRPY(-M_PI * 0.5, 0.0, -M_PI * 0.5);

    trans.frame_id_ = rectified_frame_id_;
    trans.child_frame_id_ = rectified_frame_id_ + "_optical_frame";
    trans.setRotation(to_optical);

    trans.stamp_ = image_msg->header.stamp - ros::Duration(0.001);
    tf_vec.push_back(trans);

    trans.stamp_ = image_msg->header.stamp + ros::Duration(0.001);
    tf_vec.push_back(trans);

    tfb_->sendTransform(tf_vec);
  }

  if (joint_state_pub_){
    const Eigen::Matrix3d& rot_matrix = model_->getRotationMatrix();

    tf::Matrix3x3 rot_tf;
    tf::matrixEigenToTF(rot_matrix, rot_tf);

    sensor_msgs::JointState joint_state;

    joint_state.name.push_back(yaw_joint_name_);
    joint_state.name.push_back(pitch_joint_name_);
    joint_state.position.resize(2);

    double tmp_roll;
    rot_tf.getRPY(tmp_roll, joint_state.position[1], joint_state.position[0] );

    joint_state.header.stamp = image_msg->header.stamp - ros::Duration(0.001);
    joint_state_pub_->publish(joint_state);
    joint_state.header.stamp = image_msg->header.stamp + ros::Duration(0.001);
    joint_state_pub_->publish(joint_state);
  }

}

void RectifyNodelet::configCb(Config &config, uint32_t level)
{
  config_ = config;
}

} // namespace image_proc

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( wide_angle_image_proc::RectifyNodelet, nodelet::Nodelet)

