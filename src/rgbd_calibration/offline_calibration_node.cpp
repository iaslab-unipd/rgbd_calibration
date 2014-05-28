/*
 * offline_calibration_node.cpp
 *
 *  Created on: Nov 28, 2012
 *      Author: Filippo Basso
 */

#include <ros/ros.h>

#include <pcl/io/pcd_io.h>

#include <eigen_conversions/eigen_msg.h>

#include <calibration_common/pinhole/camera_model.h>
#include <camera_info_manager/camera_info_manager.h>

#include <rgbd_calibration/offline_calibration_node.h>

using namespace camera_info_manager;
using namespace calibration_msgs;

namespace calibration
{

OfflineCalibrationNode::OfflineCalibrationNode(ros::NodeHandle & node_handle)
  : CalibrationNode(node_handle)
{

  if (not node_handle_.getParam("path", path_))
    ROS_FATAL("Missing \"path\" parameter!!");

  if (path_[path_.size() - 1] != '/')
    path_.append("/");

  if (not node_handle_.getParam("instances", instances_))
    ROS_FATAL("Missing \"instances\" parameter!!");

  node_handle_.param("image_extension", image_extension_, std::string("png"));
  node_handle_.param("starting_index", starting_index_, 1);
  node_handle_.param("image_filename", image_filename_, std::string("image_"));
  node_handle_.param("cloud_filename", cloud_filename_, std::string("cloud_"));

}

void OfflineCalibrationNode::spin()
{
  ros::Rate rate(10.0);

  int added = 0;

  ROS_INFO("Getting data...");
  for (int i = starting_index_; ros::ok() and i < starting_index_ + instances_; ++i)
  {
    std::stringstream image_file;
    image_file << path_ << image_filename_ << i << "." << image_extension_;

    std::stringstream cloud_file;
    cloud_file << path_ << cloud_filename_ << i << ".pcd";

    cv::Mat image = cv::imread(image_file.str());

    if (not image.data)
      continue;

    PCLCloud3::Ptr cloud(new PCLCloud3);
    pcl::PCDReader pcd_reader;

    if (pcd_reader.read(cloud_file.str(), *cloud) < 0)
      continue;

    calibration_->addData(image, cloud);
    ++added;

    if (added % 10 == 0)
      calibration_->addTestData(image, cloud);

    //rate.sleep();
  }

  ROS_INFO_STREAM("Added " << added << " images + point clouds.");

  geometry_msgs::Pose pose_msg;
  tf::poseEigenToMsg(color_sensor_->pose(), pose_msg);
  ROS_INFO_STREAM("Initial transform:\n" << pose_msg);

  calibration_->perform();

  tf::poseEigenToMsg(color_sensor_->pose(), pose_msg);
  ROS_INFO_STREAM("Estimated transform:\n" << pose_msg);

  calibration_->optimize();

  tf::poseEigenToMsg(color_sensor_->pose(), pose_msg);
  ROS_INFO_STREAM("Optimized transform:\n" << pose_msg);

  rate = ros::Rate(1.0);

  while (ros::ok())
  {
    calibration_->publishData();
    rate.sleep();
  }

}

} /* namespace calibration */

int main(int argc,
         char ** argv)
{
  ros::init(argc, argv, "offline_calibration");
  ros::NodeHandle node_handle("~");

  try
  {
    calibration::OfflineCalibrationNode calib_node(node_handle);
    if (not calib_node.initialize())
      return 0;
    calib_node.spin();

    ros::spin();
  }
  catch (std::runtime_error & error)
  {
    ROS_FATAL("Calibration error: %s", error.what());
    return 1;
  }

  return 0;
}

