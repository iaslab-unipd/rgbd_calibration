/*
 *  Copyright (C) 2013 - Filippo Basso <bassofil@dei.unipd.it>
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <rgbd_calibration/simulation_node.h>
#include <omp.h>

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

#include <eigen_conversions/eigen_msg.h>

#include <pcl/visualization/pcl_visualizer.h>

namespace calibration
{

SimulationNode::SimulationNode(ros::NodeHandle & node_handle)
  : CalibrationNode(node_handle)
{

  //  if (not node_handle_.getParam("instances", instances_))
  //    ROS_FATAL("Missing \"instances\" parameter!!");

  node_handle_.param("depth_error", depth_error_, 0.0035);
  node_handle_.param("image_error", image_error_, 0.5);

  node_handle_.param("min_distance", min_distance_, 1.0);
  node_handle_.param("max_distance", max_distance_, 4.0);
  node_handle_.param("step", step_, 1.0);

}

void SimulationNode::spin()
{
  ros::Rate rate(10.0);

  int added = 0;

  Scalar aa = -0.01, bb = 0.96, cc = 0.0;
  GlobalPolynomial::Coefficients global_error_function_coeffs(cc, bb, aa);
  GlobalPolynomial global_error_function(global_error_function_coeffs);

  ROS_INFO_STREAM(global_error_function_coeffs.transpose());

  ROS_INFO("Getting data...");
  for (double z = min_distance_; z <= max_distance_ + 0.01; z += step_)
  {
    Translation3 translation_cb_center(-cb_vec_[0]->center());
    Translation3 translation_z(0, 0, z);

    boost::mt19937 random_gen(time(0));

    boost::normal_distribution<> depth_error(0.0, depth_error_);
    boost::variate_generator<boost::mt19937 &, boost::normal_distribution<> > depth_noise(random_gen, depth_error);

    boost::normal_distribution<> image_error(0.0, image_error_);
    boost::variate_generator<boost::mt19937 &, boost::normal_distribution<> > image_noise(random_gen, image_error);

#pragma omp parallel for
    for (int i = 0; i < 9; ++i)
    {
      Transform T = Transform::Identity();

      if (i % 3 != 1)
      {
        AngleAxis r;
        r.axis() = Vector3::UnitY();
        r.angle() = (i % 3 == 2 ? M_PI / 6 : -M_PI / 6);
        T.prerotate(r);
      }
      if (i / 3 != 1)
      {
        AngleAxis r;
        r.axis() = Vector3::UnitX();
        r.angle() = (i / 3 == 0 ? M_PI / 6 : -M_PI / 6);
        T.prerotate(r);
      }

      T = translation_z * T * translation_cb_center;

      Checkerboard::Ptr real_cb = boost::make_shared<Checkerboard>(*cb_vec_[0]);
      real_cb->transform(T);

      // Create image

      cv::Mat image(480, 640, CV_8UC3, cv::Scalar(255, 255, 255));
      Cloud2 image_corners = color_sensor_->cameraModel()->project3dToPixel(real_cb->corners());

      std::vector<cv::Point2f> corners;
      for (Size1 c = 0; c < image_corners.elements(); ++c)
      {
        if (image_corners[c].x() > 0 and image_corners[c].x() < image.cols and image_corners[c].y() > 0
            and image_corners[c].y() < image.rows)
          corners.push_back(cv::Point2f(image_corners[c].x(), image_corners[c].y()));

        image_corners[c].x() += image_noise();
        image_corners[c].y() += image_noise();

      }
      cv::drawChessboardCorners(image, cv::Size(image_corners.size().x(), image_corners.size().y()), corners,
                                corners.size() == image_corners.elements());

      //      cv::imshow("AAA", image);
      //      cv::waitKey(100);

      // Create point cloud

      PCLCloud3::Ptr cloud = boost::make_shared<PCLCloud3>();
      std::vector<int> inliers;

      real_cb->transform(color_sensor_->pose());

      for (int y = 0; y < 480; ++y)
      {
        for (int x = 0; x < 640; ++x)
        {
          Line line(Point3::Zero(),
                    color_sensor_->cameraModel()->projectPixelTo3dRay(Point2(x, y)));
          Point3 point = line.intersectionPoint(real_cb->plane());
          Scalar new_z = (-bb + std::sqrt(bb * bb - 4 * aa * (cc - point.z()))) / (2 * aa);
          new_z = new_z + depth_noise() * new_z * new_z;
          point *= new_z / point.z();
          PCLPoint3 pcl_point;
          pcl_point.x = point.x();
          pcl_point.y = point.y();
          pcl_point.z = point.z();
          cloud->points.push_back(pcl_point);
          inliers.push_back(inliers.size());
        }
      }

      cloud->width = 640;
      cloud->height = 480;

      //      pcl::visualization::PCLVisualizer vis;
      //      vis.addPointCloud(cloud);
      //      vis.spin();

      RGBDData::Ptr data = boost::make_shared<RGBDData>(i + z * 3);
      data->setColorSensor(color_sensor_);
      data->setDepthSensor(depth_sensor_);
      data->setColorData(image);
      data->setDepthData(*cloud);

      std::stringstream ss;
      ss << "view_" << data->id() << "_" << i;

      CheckerboardViews::Ptr cb_views = boost::make_shared<CheckerboardViews>(ss.str());
      cb_views->setData(data);
      cb_views->setCheckerboard(cb_vec_[0]);
      cb_views->setImageCorners(image_corners);
      //cb_views->setPlaneInliers(inliers, 0.01);

#pragma omp critical
      {
        calibration_->addCheckerboardViews(cb_views);
        ++added;
      }
    }

    //rate.sleep();
  }

  ROS_INFO_STREAM("Added " << added << " checkerboards.");

  geometry_msgs::Pose pose_msg;
  tf::poseEigenToMsg(color_sensor_->pose(), pose_msg);
  ROS_INFO_STREAM("Initial transform:\n" << pose_msg);

  calibration_->setEstimateDepthUndistortionModel(true);
  //calibration_->setEstimateDepthUndistortionModel(false);
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
  ros::init(argc, argv, "simulation");
  ros::NodeHandle node_handle("~");

  try
  {
    calibration::SimulationNode sim_node(node_handle);
    if (not sim_node.initialize())
      return 0;
    sim_node.spin();

    ros::spin();
  }
  catch (std::runtime_error & error)
  {
    ROS_FATAL("Calibration error: %s", error.what());
    return 1;
  }

  return 0;
}

