/*
 * offline_calibration_node.cpp
 *
 *  Created on: Nov 28, 2012
 *      Author: Filippo Basso
 */

#include <boost/filesystem.hpp>

#include <ros/ros.h>

#include <pcl/io/pcd_io.h>

#include <eigen_conversions/eigen_msg.h>

#include <calibration_common/pinhole/camera_model.h>
#include <camera_info_manager/camera_info_manager.h>

#include <rgbd_calibration/offline_calibration_node.h>

using namespace camera_info_manager;
using namespace calibration_msgs;

namespace fs = boost::filesystem;

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

  node_handle_.param("image_filename", image_filename_, std::string("image_"));
  node_handle_.param("cloud_filename", cloud_filename_, std::string("cloud_"));

}

void OfflineCalibrationNode::spin()
{

  fs::path path(path_);
  fs::directory_iterator end_it;

  std::map<std::string, std::string> cloud_file_map, image_file_map;

  for (fs::directory_iterator it(path); it != end_it; ++it)
  {
    fs::path file = it->path();
    if (fs::is_regular_file(file))
    {
      std::string file_name = file.filename().string();

      if (file_name.substr(0, cloud_filename_.size()) == cloud_filename_)
      {
        std::string id = file_name.substr(cloud_filename_.size(), file_name.find_last_of('.') - cloud_filename_.size());
        cloud_file_map[id] = file.string();
      }
      else if (file_name.substr(0, image_filename_.size()) == image_filename_)
      {
        std::string id = file_name.substr(image_filename_.size(), file_name.find_last_of('.') - image_filename_.size());
        image_file_map[id] = file.string();
      }
    }
  }

  typedef std::map<std::string, std::string>::const_iterator map_iterator;

  int added = 0;

  ROS_INFO("Getting data...");
  for (map_iterator cloud_it = cloud_file_map.begin(); cloud_it != cloud_file_map.end() and added < instances_; ++cloud_it)
  {
    map_iterator image_it = image_file_map.find(cloud_it->first);
    if (image_it != image_file_map.end())
    {
      cv::Mat image = cv::imread(image_it->second);

      if (not image.data)
      {
        ROS_WARN_STREAM(image_it->second << " not valid!");
        continue;
      }

      PCLCloud3::Ptr cloud(new PCLCloud3);
      pcl::PCDReader pcd_reader;

      if (pcd_reader.read(cloud_it->second, *cloud) < 0)
      {
        ROS_WARN_STREAM(cloud_it->second << " not valid!");
        continue;
      }

      calibration_->addData(image, cloud);
      ++added;

      if (added == instances_)
        break;

      if (added % 10 == 0)
        calibration_->addTestData(image, cloud);

      ROS_DEBUG_STREAM("(" << image_it->second << ", " << cloud_it->second << ") added.");
    }
  }

  ROS_INFO_STREAM(added << " images + point clouds added.");

  geometry_msgs::Pose pose_msg;
  tf::poseEigenToMsg(color_sensor_->pose(), pose_msg);
  ROS_INFO_STREAM("Initial transform:\n" << pose_msg);

  calibration_->perform();

  tf::poseEigenToMsg(color_sensor_->pose(), pose_msg);
  ROS_INFO_STREAM("Estimated transform:\n" << pose_msg);

  calibration_->optimize();

  tf::poseEigenToMsg(color_sensor_->pose(), pose_msg);
  ROS_INFO_STREAM("Optimized transform:\n" << pose_msg);

  ros::Rate rate(1.0);

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

//    ros::spin();
  }
  catch (std::runtime_error & error)
  {
    ROS_FATAL_STREAM("Calibration error: " << error.what());
    return 1;
  }

  return 0;
}

