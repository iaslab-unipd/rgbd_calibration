/*
 *  Copyright (c) 2013-2014, Filippo Basso <bassofil@dei.unipd.it>
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *     1. Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *     2. Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *     3. Neither the name of the copyright holder(s) nor the
 *        names of its contributors may be used to endorse or promote products
 *        derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

  Size2 matrix_size_;
  Size2 images_size_;

  Calibration::Ptr calibration_;
  Publisher::Ptr publisher_;

  PinholeSensor::Ptr color_sensor_;
  KinectDepthSensor<UndistortionModel>::Ptr depth_sensor_;

};

} /* namespace calibration */
#endif /* RGBD_CALIBRATION_CALIBRATION_NODE_H_ */
