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

#ifndef RGBD_CALIBRATION_DEPTH_UNDISTORTION_ESTIMATION_H_
#define RGBD_CALIBRATION_DEPTH_UNDISTORTION_ESTIMATION_H_

#include <pcl/pcl_base.h>
#include <calibration_common/objects/checkerboard.h>
#include <calibration_common/algorithms/plane_extractor.h>
#include <calibration_common/depth/undistortion_model_fit.h>
#include <rgbd_calibration/globals.h>
#include <rgbd_calibration/publisher.h>

namespace calibration
{

class DepthUndistortionEstimation
{
public:

  struct DepthData
  {
    typedef boost::shared_ptr<DepthData> Ptr;
    typedef boost::shared_ptr<const DepthData> ConstPtr;

    DepthData()
      : plane_extracted_(false)
    {
    }

    PCLCloud3::ConstPtr cloud_;
    Checkerboard::ConstPtr checkerboard_; // Attention: in depth coordinates!!

    PCLCloud3::ConstPtr undistorted_cloud_;
    PlaneInfo estimated_plane_;

    bool plane_extracted_;

  };

  struct OrderByDistance
  {
    bool operator()(const DepthData::Ptr & lhs,
                    const DepthData::Ptr & rhs)
    {
      return lhs->checkerboard_->corners().container().row(2).maxCoeff()
          < rhs->checkerboard_->corners().container().row(2).maxCoeff();
    }
  };

  typedef boost::shared_ptr<DepthUndistortionEstimation> Ptr;
  typedef boost::shared_ptr<const DepthUndistortionEstimation> ConstPtr;

  DepthUndistortionEstimation()
    : max_threads_(1)
  {
    // Do nothing
  }

  void setLocalUndistortionModelFit(const LUndMatrixFitPCL::Ptr & local_fit)
  {
    local_fit_ = local_fit;
  }

  void setGlobalUndistortionModelFit(const GUMatrixFitPCL::Ptr & global_fit)
  {
    global_fit_ = global_fit;
    UFunctionData::Ptr data =
        boost::make_shared<UFunctionData>(GlobalPolynomial::IdentityCoefficients());
    UFunctionEigen::Ptr inv_global_und = boost::make_shared<UFunctionEigen>(data);
    inverse_global_fit_ = boost::make_shared<UFunctionFitEigen>(inv_global_und);
  }

  void addDepthData(const DepthData::Ptr & data)
  {
    data_vec_.push_back(data);
  }

  const DepthData::Ptr & addDepthData(const PCLCloud3::ConstPtr & cloud,
                                      const Checkerboard::ConstPtr & checkerboard)
  {
    DepthData::Ptr data = boost::make_shared<DepthData>();
    data->cloud_ = cloud;
    data->checkerboard_ = checkerboard;
    addDepthData(data);
    return data_vec_.back();
  }

  void setDepthData(const std::vector<DepthData::Ptr> & data_vec)
  {
    data_vec_ = data_vec;
  }

  //  const std::vector<DepthData::Ptr> & depthData()
  //  {
  //    return data_vec_;
  //  }

  void estimateLocalModel();

  void estimateGlobalModel();

  void setMaxThreads(size_t max_threads)
  {
    assert(max_threads > 0);
    max_threads_ = max_threads;
  }

  Eigen::Matrix<Scalar, Eigen::Dynamic, 3> getLocalSamples(size_t x_index,
                                                           size_t y_index) const
  {
    const std::vector<std::pair<Scalar, Scalar> > & samples = local_fit_->getSamples(x_index, y_index);

    Eigen::Matrix<Scalar, Eigen::Dynamic, 3> eigen_samples(samples.size(), 3);

    for (size_t i = 0; i < samples.size(); ++i)
      eigen_samples.row(i) << samples[i].first, samples[i].second, samples[i].first - samples[i].second;

    return eigen_samples;
  }

  Eigen::Matrix<Scalar, Eigen::Dynamic, 3> getGlobalSamples(size_t x_index,
                                                            size_t y_index) const
  {
    const std::vector<std::pair<Scalar, Scalar> > & samples = global_fit_->getSamples(x_index, y_index);

    Eigen::Matrix<Scalar, Eigen::Dynamic, 3> eigen_samples(samples.size(), 3);

    for (size_t i = 0; i < samples.size(); ++i)
      eigen_samples.row(i) << samples[i].first, samples[i].second, samples[i].first - samples[i].second;

    return eigen_samples;
  }

private:

  bool extractPlane(const Checkerboard & color_cb,
                    const PCLCloud3::ConstPtr & cloud,
                    const Point3 & color_cb_center,
                    PlaneInfo & plane_info);

  size_t max_threads_;

  LUndMatrixFitPCL::Ptr local_fit_;
  GUMatrixFitPCL::Ptr global_fit_;
  UFunctionFitEigen::Ptr inverse_global_fit_;

  std::vector<DepthData::Ptr> data_vec_;

};

} /* namespace calibration */
#endif /* RGBD_CALIBRATION_DEPTH_UNDISTORTION_ESTIMATION_H_ */
