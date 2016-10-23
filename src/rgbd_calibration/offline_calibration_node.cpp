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

#include <kinect/depth/polynomial_matrix_io.h>
#include <rgbd_calibration/offline_calibration_node.h>

//#include <swissranger_camera/utility.h>
#include <pcl/conversions.h>

using namespace camera_info_manager;
using namespace calibration_msgs;

namespace fs = boost::filesystem;

namespace calibration
{

OfflineCalibrationNode::OfflineCalibrationNode (ros::NodeHandle & node_handle)
  : CalibrationNode(node_handle),
	starting_index_(0)
{
  if (not node_handle_.getParam("path", path_))
    ROS_FATAL("Missing \"path\" parameter!!");

  if (path_[path_.size() - 1] != '/')
    path_.append("/");

  if (not node_handle_.getParam("instances", instances_))
    ROS_FATAL("Missing \"instances\" parameter!!");

  node_handle_.param("image_filename", image_filename_, std::string("image_"));
  node_handle_.param("cloud_filename", cloud_filename_, std::string("cloud_"));

  std::string depth_type_s;
  node_handle_.param("depth_type", depth_type_s, std::string("none"));
  if (depth_type_s == "kinect1_depth")
    depth_type_ = KINECT1_DEPTH;
  else if (depth_type_s == "swiss_ranger_depth")
    depth_type_ = SWISS_RANGER_DEPTH;
  else
    ROS_FATAL("Missing \"depth_type\" parameter!! Use \"kinect1_depth\" or \"swiss_ranger_depth\"");
}

void
OfflineCalibrationNode::spin ()
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

      PCLCloud3::Ptr cloud;

      pcl::PCDReader pcd_reader;
      if (depth_type_ == KINECT1_DEPTH)
      {
        cloud = boost::make_shared<PCLCloud3>();

        if (pcd_reader.read(cloud_it->second, *cloud) < 0)
        {
          ROS_WARN_STREAM(cloud_it->second << " not valid!");
          continue;
        }

        const Scalar & fx = depth_sensor_->cameraModel()->fx() * depth_sensor_->cameraModel()->binningX();
        const Scalar & fy = depth_sensor_->cameraModel()->fy() * depth_sensor_->cameraModel()->binningY();
    	const Scalar & cx = depth_sensor_->cameraModel()->cx() * depth_sensor_->cameraModel()->binningX();
    	const Scalar & cy = depth_sensor_->cameraModel()->cy() * depth_sensor_->cameraModel()->binningY();

        for (int k = 0; k < cloud->width; ++k)
        {
    		for (Size1 j = 0; j < cloud->height; ++j)
    		{
    		  cloud->at(k, j).x = (k - cx) * cloud->at(k, j).z / fx;
    		  cloud->at(k, j).y = (j - cy) * cloud->at(k, j).z / fy;
    		}
        }

        /*for (size_t v = 0; v < cloud->height; ++v)
        {
          for (size_t u = 0; u < cloud->width; ++u)
          {
            PCLPoint3 & pt = cloud->points[u + v * cloud->width];
//            pt.x = (u - 314.5) * pt.z / 575.8157348632812;
//            pt.y = (v - 235.5) * pt.z / 575.8157348632812;
//            pt.x = (u - 309.7947658766498) * pt.z / 584.3333129882812;
//            pt.y = (v - 245.9642466885198) * pt.z / 582.8702392578125;
            pt.x = (u - depth_sensor_->cameraModel()->cx()) * pt.z / depth_sensor_->cameraModel()->fx();
            pt.y = (v - depth_sensor_->cameraModel()->cy()) * pt.z / depth_sensor_->cameraModel()->fy();
          }
        }*/

//        pcl::PCDWriter pcd_writer;
//        pcd_writer.write(cloud_it->second + "_rev.pcd", *cloud);

      }
      else if (depth_type_ == SWISS_RANGER_DEPTH)
      {
//        pcl::PCLPointCloud2Ptr pcl_cloud = boost::make_shared<pcl::PCLPointCloud2>();
//        sensor_msgs::PointCloud2Ptr ros_cloud = boost::make_shared<sensor_msgs::PointCloud2>();
//        sr::Utility sr_utility;
//        if (pcd_reader.read(cloud_it->second, *pcl_cloud) < 0)
//        {
//          ROS_WARN_STREAM(cloud_it->second << " not valid!");
//          continue;
//        }
//        pcl_conversions::fromPCL(*pcl_cloud, *ros_cloud);
//
//        sr_utility.setConfidenceThreshold(0.90f);
//        sr_utility.setInputCloud(ros_cloud);
//        sr_utility.setIntensityType(sr::Utility::INTENSITY_8BIT);
//        sr_utility.setConfidenceType(sr::Utility::CONFIDENCE_8BIT);
//        sr_utility.setNormalizeIntensity(true);
//        sr_utility.split(sr::Utility::CLOUD);
//
//        cloud = sr_utility.cloud();
      }

      calibration_->addData(image, cloud);
      ++added;

      if (added == instances_)
        break;

      ROS_DEBUG_STREAM("(" << image_it->second << ", " << cloud_it->second << ") added.");
    }
  }

  ROS_INFO_STREAM(added << " images + point clouds added.");

  geometry_msgs::Pose pose_msg;
  tf::poseEigenToMsg(color_sensor_->pose(), pose_msg);
  ROS_INFO_STREAM("Initial transform:\n" << pose_msg);

  calibration_->perform();

  PolynomialUndistortionMatrixIO<LocalPolynomial> local_io;
  local_io.write(*calibration_->localModel(), path_ + "local_matrix.txt");

  tf::poseEigenToMsg(color_sensor_->pose(), pose_msg);
  ROS_INFO_STREAM("Estimated transform:\n" << pose_msg);

  calibration_->optimize();

  PolynomialUndistortionMatrixIO<GlobalPolynomial> global_io;
  global_io.write(*calibration_->globalModel(), path_ + "global_matrix.txt");

  tf::poseEigenToMsg(color_sensor_->pose(), pose_msg);
  ROS_INFO_STREAM("Optimized transform:\n" << pose_msg);

  std::ofstream transform_file;
  transform_file.open((path_ + "camera_pose.yaml").c_str());
  transform_file << pose_msg;
  transform_file.close();

  const std::vector<double> & depth_intrinsics = calibration_->optimizedIntrinsics();

  ROS_INFO_STREAM("fx = " << depth_sensor_->cameraModel()->binningX() * depth_intrinsics[0]);
  ROS_INFO_STREAM("fy = " << depth_sensor_->cameraModel()->binningY() * depth_intrinsics[1]);
  ROS_INFO_STREAM("cx = " << depth_sensor_->cameraModel()->binningX() * depth_intrinsics[2]);
  ROS_INFO_STREAM("cy = " << depth_sensor_->cameraModel()->binningY() * depth_intrinsics[3]);

  std::ofstream intrinsics_file;
  intrinsics_file.open((path_ + "depth_intrinsics.yaml").c_str());
  intrinsics_file << "intrinsics:" << std::endl;
  intrinsics_file << "  fx: " << depth_sensor_->cameraModel()->binningX() * depth_intrinsics[0] << std::endl;
  intrinsics_file << "  fy: " << depth_sensor_->cameraModel()->binningY() * depth_intrinsics[1] << std::endl;
  intrinsics_file << "  cx: " << depth_sensor_->cameraModel()->binningX() * depth_intrinsics[2] << std::endl;
  intrinsics_file << "  cy: " << depth_sensor_->cameraModel()->binningY() * depth_intrinsics[3] << std::endl;
  intrinsics_file.close();


//  ros::Rate rate(1.0);

//  while (ros::ok())
//  {
//    calibration_->publishData();
//    rate.sleep();
//  }

}

} /* namespace calibration */

int
main (int argc,
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
  }
  catch (const std::runtime_error & error)
  {
    ROS_FATAL_STREAM("Calibration error: " << error.what());
    return 1;
  }

  return 0;
}

