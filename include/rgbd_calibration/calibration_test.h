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

#ifndef RGBD_CALIBRATION_CALIBRATION_TEST_H_
#define RGBD_CALIBRATION_CALIBRATION_TEST_H_

#include <calibration_common/pinhole/sensor.h>
#include <kinect/depth/sensor.h>

#include <rgbd_calibration/globals.h>
#include <rgbd_calibration/publisher.h>
#include <rgbd_calibration/checkerboard_views.h>

//#define HERRERA
//#define UNCALIBRATED
//#define SKEW

namespace calibration
{

class CalibrationTest
{
public:

  typedef boost::shared_ptr<CalibrationTest> Ptr;
  typedef boost::shared_ptr<const CalibrationTest> ConstPtr;

  void setColorSensor(const PinholeSensor::Ptr & color_sensor)
  {
    color_sensor_ = color_sensor;
  }

  void setDepthSensor(const KinectDepthSensorBase::Ptr & depth_sensor)
  {
    depth_sensor_ = depth_sensor;
  }

  void setCheckerboards(const std::vector<Checkerboard::ConstPtr> & cb_vec)
  {
    cb_vec_ = cb_vec;
  }

  void setPublisher(const Publisher::Ptr & publisher)
  {
    publisher_ = publisher;
  }

  void setDownSampleRatio(int ratio)
  {
    assert(ratio > 0);
    ratio_ = ratio;
  }

  PCLCloudRGB::Ptr addData(const cv::Mat & image,
                           const PCLCloud3::ConstPtr & cloud);

  void setLocalModel(const LocalModel::Ptr & model)
  {
    local_matrix_ = boost::make_shared<LocalMatrixPCL>(model);
  }

  void setGlobalModel(const GlobalModel::Ptr & model)
  {
    global_matrix_ = boost::make_shared<GlobalMatrixPCL>(model);
  }

  void publishData() const;

  void visualizeClouds() const;

  void testPlanarityError() const;

  void testCheckerboardError() const;

  void testCube() const;

protected:

  boost::shared_ptr<std::vector<int> >
  extractPlaneFromCloud (const PCLCloud3::Ptr & cloud,
                         const Cloud2 & depth_corners) const;

  PinholeSensor::Ptr color_sensor_;
  KinectDepthSensorBase::Ptr depth_sensor_;

  std::vector<Checkerboard::ConstPtr> cb_vec_;

  Publisher::Ptr publisher_;

  int ratio_;

  LocalMatrixPCL::Ptr local_matrix_;
  GlobalMatrixPCL::Ptr global_matrix_;

public:

  std::vector<RGBDData::ConstPtr> data_vec_;
  std::vector<RGBDData::ConstPtr> part_und_data_vec_;
  std::vector<RGBDData::ConstPtr> und_data_vec_;

  std::map<RGBDData::ConstPtr, RGBDData::ConstPtr> data_map_;
  std::map<RGBDData::ConstPtr, RGBDData::ConstPtr> part_data_map_;

};

} /* namespace calibration */
#endif /* RGBD_CALIBRATION_CALIBRATION_TEST_H_ */
