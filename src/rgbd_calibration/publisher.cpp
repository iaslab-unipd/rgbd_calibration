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

#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen_conversions/eigen_msg.h>
#include <cv_bridge/cv_bridge.h>

#include <rgbd_calibration/publisher.h>

namespace calibration
{

Publisher::Publisher(const ros::NodeHandle & node_handle)
  : node_handle_(node_handle)
{
  marker_pub_ = node_handle_.advertise<visualization_msgs::Marker>("rgbd_markers", 0);
}

void Publisher::publishTF(const BaseObject & object)
{
  geometry_msgs::TransformStamped transform_msg;
  object.toTF(transform_msg);
  tf_pub_.sendTransform(transform_msg);
}

//void Publisher::publish(const RGBDCheckerboard & rgbd_checkerboard,
//                            const DistortionMap<double, PointT> & map)
//{
//  publish(rgbd_checkerboard);
//
//  visualization_msgs::Marker cloud_marker;
//  cloud_marker.ns = "undist_cloud";
//
//  Point3Matrix points = rgbd_checkerboard.depthView()->interestPoints();
//  map.undistort(points);
//
//  cloud_marker.header.stamp = ros::Time::now();
//  cloud_marker.header.frame_id = rgbd_checkerboard.depthView()->sensor()->frameId();
//  points.toMarker(cloud_marker);
//  cloud_marker.color.r = 0.0;
//  cloud_marker.color.g = 1.0;
//  cloud_marker.color.b = 0.0;
//
//  CheckerboardPublisherSet & pub_set = c_pub_map_[rgbd_checkerboard.id()];
//  pub_set.marker_pub_.publish(cloud_marker);
//}

void Publisher::publish(const CheckerboardViews & rgbd_checkerboard,
                        const std::string & prefix)
{
  std::stringstream ss;
  ss << prefix << "checkerboard_" << rgbd_checkerboard.id();
  std::string ns = ss.str();

  if (c_pub_map_.find(ns) == c_pub_map_.end())
  {
    CheckerboardPublisherSet pub_set;

    std::stringstream sss;
    sss << ns << "/markers";
    pub_set.marker_pub_ = node_handle_.advertise<visualization_msgs::Marker>(sss.str(), 0);

    c_pub_map_[ns] = pub_set;
  }

  visualization_msgs::Marker checkerboard_marker;
  visualization_msgs::Marker cloud_marker;
  visualization_msgs::Marker d_plane_marker;

  checkerboard_marker.ns = "checkerboard";
  cloud_marker.ns = "cloud";
  d_plane_marker.ns = "d_plane";

  checkerboard_marker.id = cloud_marker.id = d_plane_marker.id = 0;

  CheckerboardPublisherSet & pub_set = c_pub_map_[ns];

  if (rgbd_checkerboard.colorCheckerboard())
  {
    rgbd_checkerboard.colorCheckerboard()->toMarker(checkerboard_marker);
    pub_set.marker_pub_.publish(checkerboard_marker);
  }

//  if (rgbd_checkerboard.depthPlane())
//  {
//    rgbd_checkerboard.depthPlane()->toMarker(d_plane_marker);
//    pub_set.marker_pub_.publish(d_plane_marker);
//  }

//  if (rgbd_checkerboard.depthView())
//  {
//    rgbd_checkerboard.depthView()->toMarker(cloud_marker);
//    pub_set.marker_pub_.publish(cloud_marker);
//  }
}

void Publisher::publish(const RGBDData & rgbd)
{
  std::stringstream ss;
  ss << "rgbd_" << rgbd.id();
  std::string ns = ss.str();

  if (d_pub_map_.find(rgbd.id()) == d_pub_map_.end())
  {
    DataPublisherSet pub_set;

    ss.str("");
    ss << ns << "/cloud";
    pub_set.cloud_pub_ = node_handle_.advertise<PCLCloud3>(ss.str(), 0);

    //    ss.str("");
    //    ss << ns << "/image";
    //    pub_set.image_pub_ = node_handle_.advertise<sensor_msgs::Image>(ss.str(), 0);

    ss.str("");
    ss << ns << "/rgbd";
    pub_set.rgbd_pub_ = node_handle_.advertise<PCLCloudRGB>(ss.str(), 0);

    d_pub_map_[rgbd.id()] = pub_set;
  }

  DataPublisherSet & pub_set = d_pub_map_[rgbd.id()];

  rgbd.depthData()->header.stamp = ros::Time::now().toNSec();
  pub_set.cloud_pub_.publish(*rgbd.depthData());
  rgbd.fuseData();
  pub_set.rgbd_pub_.publish(*rgbd.fusedData());

}

} /* namespace calibration */
