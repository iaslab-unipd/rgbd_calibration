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

#ifndef RGBD_CALIBRATION_PUBLISHER_H_
#define RGBD_CALIBRATION_PUBLISHER_H_

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <rgbd_calibration/checkerboard_views.h>

namespace calibration
{

class Publisher
{
private:

  struct CheckerboardPublisherSet
  {
    ros::Publisher marker_pub_;
  };

  struct DataPublisherSet
  {
    ros::Publisher cloud_pub_;
    ros::Publisher rgbd_pub_;
  };

public:

  typedef boost::shared_ptr<Publisher> Ptr;
  typedef boost::shared_ptr<const Publisher> ConstPtr;

  Publisher(const ros::NodeHandle & node_handle);

  void publish(const CheckerboardViews & rgbd,
               const std::string & prefix = "");

  void publish(const RGBDData & rgbd);

  void publish(const Cloud3 & point_set,
               const BaseObject & frame);

  void publishTF(const BaseObject & object);

private:

  ros::NodeHandle node_handle_;
  ros::Publisher marker_pub_;
  tf::TransformBroadcaster tf_pub_;

  std::map<int, DataPublisherSet> d_pub_map_;
  std::map<std::string, CheckerboardPublisherSet> c_pub_map_;

};

} /* namespace calibration */
#endif /* RGBD_CALIBRATION_PUBLISHER_H_ */
