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

  inline void setColorSensor(const PinholeSensor::Ptr & color_sensor)
  {
    color_sensor_ = color_sensor;
  }

  inline void setDepthSensor(const KinectDepthSensor<UndistortionModel>::Ptr & depth_sensor)
  {
    depth_sensor_ = depth_sensor;
  }

  inline void setCheckerboards(const std::vector<Checkerboard::ConstPtr> & cb_vec)
  {
    cb_vec_ = cb_vec;
  }

  inline void setPublisher(const Publisher::Ptr & publisher)
  {
    publisher_ = publisher;
  }

  inline void setDownSampleRatio(int ratio)
  {
    assert(ratio > 0);
    ratio_ = ratio;
  }

  void addData(const cv::Mat & image,
               const PCLCloud3::ConstPtr & cloud);

  void addTestData(const cv::Mat & image,
                   const PCLCloud3::ConstPtr & cloud);

  inline void setEstimateInitialTransform(bool estimate_initial_trasform)
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

    assert(local_matrix_ and global_matrix_);
    estimate_depth_und_model_ = true;

    depth_undistortion_estimation_ = boost::make_shared<DepthUndistortionEstimation>();
    depth_undistortion_estimation_->setLocalModel(local_model_);
    depth_undistortion_estimation_->setGlobalModel(global_model_);
    depth_undistortion_estimation_->setMaxThreads(8);
  }

  inline void setForceAll(bool force)
  {
    force_ = force;
  }

  inline void addCheckerboardViews(const CheckerboardViews::Ptr & rgbd_cb)
  {
    cb_views_vec_.push_back(rgbd_cb);
  }

  inline void setLocalModel(const LocalModel::Ptr & model)
  {
    local_model_ = model;
    local_matrix_ = boost::make_shared<LocalMatrixPCL>(local_model_);
  }

  inline void setGlobalModel(const GlobalModel::Ptr & model)
  {
    global_model_ = model;
    global_matrix_ = boost::make_shared<GlobalMatrixPCL>(global_model_);
  }

  void perform();

  void optimize();

  void publishData() const;

protected:

  void estimateInitialTransform();

  void estimateTransform(const std::vector<CheckerboardViews::Ptr> & rgbd_cb_vec);

  void optimizeTransform(const std::vector<CheckerboardViews::Ptr> & rgbd_cb_vec);

  void optimizeAll(const std::vector<CheckerboardViews::Ptr> & rgbd_cb_vec);

  void addData_(const cv::Mat & image,
                const PCLCloud3::ConstPtr & cloud,
                std::vector<RGBDData::ConstPtr> & vec);

  PinholeSensor::Ptr color_sensor_;
  KinectDepthSensor<UndistortionModel>::Ptr depth_sensor_;

  std::vector<Checkerboard::ConstPtr> cb_vec_;

  Publisher::Ptr publisher_;

  bool estimate_depth_und_model_;
  bool estimate_initial_trasform_;
  bool force_;
  int ratio_;

  LocalModel::Ptr local_model_;
  GlobalModel::Ptr global_model_;

  LocalMatrixPCL::Ptr local_matrix_;
  GlobalMatrixPCL::Ptr global_matrix_;

  DepthUndistortionEstimation::Ptr depth_undistortion_estimation_;

  std::vector<RGBDData::ConstPtr> data_vec_;
  std::vector<RGBDData::ConstPtr> test_vec_;

  std::vector<CheckerboardViews::Ptr> cb_views_vec_;
  std::vector<DepthUndistortionEstimation::DepthData::Ptr> depth_data_vec_;

};

} /* namespace calibration */
#endif /* RGBD_CALIBRATION_CALIBRATION_H_ */
