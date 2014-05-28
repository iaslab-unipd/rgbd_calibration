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

#ifndef RGBD_CALIBRATION_CALIBRATION_H_
#define RGBD_CALIBRATION_CALIBRATION_H_

#include <calibration_common/pinhole/sensor.h>

#include <kinect/depth/sensor.h>

#include <rgbd_calibration/depth_undistortion_estimation.h>
#include <rgbd_calibration/publisher.h>
#include <rgbd_calibration/checkerboard_views.h>
#include <rgbd_calibration/plane_based_extrinsic_calibration.h>

namespace calibration
{

class Calibration
{
public:

  typedef boost::shared_ptr<Calibration> Ptr;
  typedef boost::shared_ptr<const Calibration> ConstPtr;

  void setColorSensor(const PinholeSensor::Ptr & color_sensor)
  {
    color_sensor_ = color_sensor;
  }

  void setDepthSensor(const KinectDepthSensor<UndistortionModel>::Ptr & depth_sensor)
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

  void addData(const cv::Mat & image,
               const PCLCloud3::ConstPtr & cloud);

  void addTestData(const cv::Mat & image,
                   const PCLCloud3::ConstPtr & cloud);

  void setEstimateInitialTransform(bool estimate_initial_trasform)
  {
    estimate_initial_trasform_ = estimate_initial_trasform;
  }

  void setEstimateDepthUndistortionModel(bool estimate_depth_und_model)
  {
    if (not estimate_depth_und_model)
    {
      estimate_depth_und_model_ = false;
      return;
    }

    assert(local_und_ and global_und_);
    estimate_depth_und_model_ = true;

    LUndMatrixFitPCL::Ptr local_fit = boost::make_shared<LUndMatrixFitPCL>(local_und_);
    GUMatrixFitPCL::Ptr global_fit = boost::make_shared<GUMatrixFitPCL>(global_und_);

    depth_undistortion_estimation_ = boost::make_shared<DepthUndistortionEstimation>();
    depth_undistortion_estimation_->setLocalUndistortionModelFit(local_fit);
    depth_undistortion_estimation_->setGlobalUndistortionModelFit(global_fit);
    depth_undistortion_estimation_->setMaxThreads(8);
  }

  void setForceAll(bool force)
  {
    force_ = force;
  }

  void addCheckerboardViews(const CheckerboardViews::Ptr & rgbd_cb)
  {
    cb_views_vec_.push_back(rgbd_cb);
  }

  void setUndistortionModels(const LUMatrixPCL::Ptr & local_und,
                             const GUMatrixPCL::Ptr & global_und)
  {
    local_und_ = local_und;
    global_und_ = global_und;
  }

  void setLocalUndistortionModel(const LUMatrixModel::Ptr & model)
  {
    local_und_ = boost::make_shared<LUMatrixPCL>(model);
  }

  void setGlobalUndistortionModel(const GUMatrixModel::Ptr & model)
  {
    global_und_ = boost::make_shared<GUMatrixPCL>(model);
  }

  void perform();

  void optimize();

  void publishData() const;

protected:

  void estimateInitialTransform();

  void estimateTransform(const std::vector<CheckerboardViews::Ptr> & rgbd_cb_vec);

  void optimizeTransform(const std::vector<CheckerboardViews::Ptr> & rgbd_cb_vec);

  void optimizeAll(const std::vector<CheckerboardViews::Ptr> & rgbd_cb_vec);

  PinholeSensor::Ptr color_sensor_;
  KinectDepthSensor<UndistortionModel>::Ptr depth_sensor_;

  std::vector<Checkerboard::ConstPtr> cb_vec_;

  Publisher::Ptr publisher_;

  bool estimate_depth_und_model_;
  bool estimate_initial_trasform_;
  bool force_;
  int ratio_;

  LUMatrixPCL::Ptr local_und_;
  GUMatrixPCL::Ptr global_und_;

  DepthUndistortionEstimation::Ptr depth_undistortion_estimation_;

  std::vector<RGBDData::ConstPtr> data_vec_;
  std::vector<RGBDData::ConstPtr> test_vec_;

  std::vector<CheckerboardViews::Ptr> cb_views_vec_;
  std::vector<DepthUndistortionEstimation::DepthData::Ptr> depth_data_vec_;

};

} /* namespace calibration */
#endif /* RGBD_CALIBRATION_CALIBRATION_H_ */
