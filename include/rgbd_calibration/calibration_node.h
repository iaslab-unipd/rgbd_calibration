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

#ifndef RGBD_CALIBRATION_CALIBRATION_NODE_H_
#define RGBD_CALIBRATION_CALIBRATION_NODE_H_

#include <ros/ros.h>

#include <calibration_msgs/CheckerboardArray.h>

#include <calibration_common/base/base.h>
#include <calibration_common/objects/checkerboard.h>

#include <kinect/depth/sensor.h>

#include <rgbd_calibration/globals.h>
#include <rgbd_calibration/calibration.h>

namespace calibration
{

class CalibrationNode
{
public:

  CalibrationNode(ros::NodeHandle & node_handle);

  virtual ~CalibrationNode()
  {
    // Do nothing
  }

  virtual bool initialize();

  virtual void spin() = 0;

  static Checkerboard::Ptr createCheckerboard(const calibration_msgs::CheckerboardMsg::ConstPtr & msg,
                                              int id);

  static Checkerboard::Ptr createCheckerboard(const calibration_msgs::CheckerboardMsg & msg,
                                              int id);

protected:

  void checkerboardArrayCallback(const calibration_msgs::CheckerboardArray::ConstPtr & msg);

  bool waitForMessages() const;

  ros::NodeHandle node_handle_;

  // TODO find another way to get checkerboards
  ros::Subscriber checkerboards_sub_;
  calibration_msgs::CheckerboardArray::ConstPtr checkerboard_array_msg_;

  std::vector<Checkerboard::ConstPtr> cb_vec_;

  std::string camera_calib_url_;
  std::string camera_name_;

  bool has_initial_transform_;
  Transform initial_transform_;

  int downsample_ratio_;
  bool estimate_depth_distortion_;
  int distortion_cols_;
  int distortion_rows_;

  Calibration::Ptr calibration_;
  Publisher::Ptr publisher_;

  PinholeSensor::Ptr color_sensor_;
  KinectDepthSensor<UndistortionModel>::Ptr depth_sensor_;

};

} /* namespace calibration */
#endif /* RGBD_CALIBRATION_CALIBRATION_NODE_H_ */
