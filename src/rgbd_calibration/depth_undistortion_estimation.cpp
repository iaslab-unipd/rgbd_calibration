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

#include <ros/ros.h>
#include <omp.h>
#include <algorithm>

#include <calibration_common/ceres/polynomial_fit.h>
#include <calibration_common/ceres/plane_fit.h>
#include <calibration_common/base/pcl_conversion.h>

#include <rgbd_calibration/checkerboard_views.h>
#include <calibration_common/algorithms/plane_extraction.h>
#include <rgbd_calibration/depth_undistortion_estimation.h>

#define RGBD_INFO(id, msg) ROS_INFO_STREAM("RGBD " << id << ": " << msg)
#define RGBD_WARN(id, msg) ROS_WARN_STREAM("RGBD " << id << ": " << msg)

namespace calibration
{

bool DepthUndistortionEstimation::extractPlane(const Checkerboard & color_cb,
                                               const PCLCloud3::ConstPtr & cloud,
                                               const Point3 & center,
                                               PlaneInfo & plane_info)
{
  Scalar radius = std::min(color_cb.width(), color_cb.height()) / 1.8; // TODO Add parameter
  PointPlaneExtraction<PCLPoint3> plane_extractor;
  plane_extractor.setInputCloud(cloud);
  plane_extractor.setRadius(radius);

  bool plane_extracted = false;

  int k[5] = {0, 1, -1, 2, -2}; // TODO Add parameter
  for (size_t i = 0; i < 5 and not plane_extracted; ++i)
  {
    plane_extractor.setPoint(PCLPoint3(center.x(), center.y(), center.z() + radius * k[i]));
    plane_extracted = plane_extractor.extract(plane_info);
  }

  return plane_extracted;
}

void DepthUndistortionEstimation::estimateLocalModel()
{
  std::sort(data_vec_.begin(), data_vec_.end(), OrderByDistance());

  for (size_t i = 0; i < data_vec_.size(); i += max_threads_)
  {
#pragma omp parallel for schedule(static, 1)
    for (size_t th = 0; th < max_threads_; ++th)
    {
      if (i + th >= data_vec_.size())
        continue;

      const DepthData & data = *data_vec_[i + th];
      const Checkerboard & gt_cb = *data.checkerboard_;
      const PCLCloud3 & cloud = *data.cloud_;

      // Estimate center
      Point3 und_color_cb_center = gt_cb.center();
#pragma omp critical
      {
        InverseGlobalMatrixEigen inverse_global(inverse_global_fit_->model());
        inverse_global.undistort(0, 0, und_color_cb_center);
      }

      RGBD_INFO(i + th, "Transformed z: " << gt_cb.center().z() << " -> " << und_color_cb_center.z());

      // Undistort cloud
      PCLCloud3::Ptr und_cloud = boost::make_shared<PCLCloud3>(cloud);
#pragma omp critical
      {
        LocalMatrixPCL local(local_fit_->model());
        local.undistort(*und_cloud);
      }

      // Extract plane from undistorted cloud
      PlaneInfo plane_info;
      if (extractPlane(gt_cb, und_cloud, und_color_cb_center, plane_info))
      {
        RGBD_INFO(i + th, "Plane extracted!!");

        Plane fitted_plane = PlaneFit<Scalar>::robustFit(PCLConversion<Scalar>::toPointMatrix(cloud, *plane_info.indices_),
                                                         plane_info.std_dev_);

#pragma omp critical
        {
          local_fit_->accumulateCloud(cloud, *plane_info.indices_);
          local_fit_->addAccumulatedPoints(fitted_plane);
          for (Size1 c = 0; c < gt_cb.corners().elements(); ++c)
          {
            const Point3 & corner = gt_cb.corners()[c];
            inverse_global_fit_->addPoint(0, 0, corner, plane_info.plane_);
          }
          if (i + th > 20)
            inverse_global_fit_->update();
        }

        Line line(gt_cb.center(), Point3::UnitZ());
        RGBD_INFO(i + th, "Real z: " << line.intersectionPoint(plane_info.plane_).z());

//      Scalar angle = RAD2DEG(std::acos(plane_info.equation_.normal().dot(gt_cb.plane().normal())));
//      RGBD_INFO(data.id(), "Angle: " << angle);

      }
      else
        RGBD_WARN(i + th, "Plane not extracted!!");

    }

    local_fit_->update();
  }

}

void DepthUndistortionEstimation::estimateGlobalModel()
{
#pragma omp parallel for
  for (size_t i = 0; i < data_vec_.size(); ++i)
  {
    DepthData & data = *data_vec_[i];
    const Checkerboard & gt_cb = *data.checkerboard_;
    const PCLCloud3 & cloud = *data.cloud_;

    Point3 und_color_cb_center = gt_cb.center();
    InverseGlobalMatrixEigen inverse_global(inverse_global_fit_->model());
    inverse_global.undistort(0, 0, und_color_cb_center);

    PCLCloud3::Ptr und_cloud = boost::make_shared<PCLCloud3>(cloud);
    LocalMatrixPCL local(local_fit_->model());
    local.undistort(*und_cloud);

    PlaneInfo plane_info;

    if (extractPlane(gt_cb, und_cloud, und_color_cb_center, plane_info))
    {
      data.estimated_plane_ = plane_info;
      data.undistorted_cloud_ = und_cloud;
      data.plane_extracted_ = true;

#pragma omp critical
      {
        global_fit_->accumulateCloud(*und_cloud, *plane_info.indices_);
        global_fit_->addAccumulatedPoints(gt_cb.plane());
      }
    }
    else
      RGBD_WARN(i, "Plane not extracted!!");

  }

  global_fit_->update();

}

} /* namespace calibration */
