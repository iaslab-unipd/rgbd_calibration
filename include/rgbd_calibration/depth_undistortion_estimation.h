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

#ifndef RGBD_CALIBRATION_DEPTH_UNDISTORTION_ESTIMATION_H_
#define RGBD_CALIBRATION_DEPTH_UNDISTORTION_ESTIMATION_H_

#include <pcl/pcl_base.h>
#include <calibration_common/objects/checkerboard.h>
#include <calibration_common/algorithms/plane_extraction.h>
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

    DepthData(int id)
      : id_(id),
        plane_extracted_(false)
    {
    }

    int id_;

    PCLCloud3::ConstPtr cloud_;
    Checkerboard::ConstPtr checkerboard_; // Attention: in depth coordinates!!

    PCLCloud3::ConstPtr undistorted_cloud_;
    PlaneInfo estimated_plane_;

    bool plane_extracted_;

  };

  struct OrderByDistance
  {
    inline bool operator()(const DepthData::Ptr & lhs,
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

  inline void setLocalModel(const LocalModel::Ptr & local_model)
  {
    local_model_ = local_model;
    local_fit_ = boost::make_shared<LocalMatrixFitPCL>(local_model_);
    local_fit_->setDepthErrorFunction(depth_error_function_);
  }

  inline void setGlobalModel(const GlobalModel::Ptr & global_model)
  {
    global_model_ = global_model;
    global_fit_ = boost::make_shared<GlobalMatrixFitPCL>(global_model_);
    global_fit_->setDepthErrorFunction(depth_error_function_);
    InverseGlobalModel::Data::Ptr matrix = boost::make_shared<InverseGlobalModel::Data>(Size2(1, 1), GlobalPolynomial::IdentityCoefficients());
    inverse_global_model_ = boost::make_shared<InverseGlobalModel>(global_model_->imageSize());
    inverse_global_model_->setMatrix(matrix);
    inverse_global_fit_ = boost::make_shared<InverseGlobalMatrixFitEigen>(inverse_global_model_);
  }

  inline void addDepthData(const DepthData::Ptr & data)
  {
    data_vec_.push_back(data);
  }

  inline const DepthData::Ptr & addDepthData(const PCLCloud3::ConstPtr & cloud,
                                             const Checkerboard::ConstPtr & checkerboard)
  {
    DepthData::Ptr data = boost::make_shared<DepthData>(data_vec_.size() + 1);
    data->cloud_ = cloud;
    data->checkerboard_ = checkerboard;
    addDepthData(data);
    return data_vec_.back();
  }

  inline void setDepthData(const std::vector<DepthData::Ptr> & data_vec)
  {
    data_vec_ = data_vec;
  }

  void estimateLocalModel();

  void estimateLocalModelReverse();

  void optimizeLocalModel(const Polynomial<double, 2> & depth_error_function);

  void estimateGlobalModel();

  inline void setMaxThreads(size_t max_threads)
  {
    assert(max_threads > 0);
    max_threads_ = max_threads;
  }

  Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> getLocalSamples(Size1 x_index,
                                                                        Size1 y_index) const
  {
    const LocalMatrixFitPCL::DataBin & samples = local_fit_->getSamples(x_index, y_index);
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> eigen_samples(samples.size(), 3);

    for (Size1 i = 0; i < samples.size(); ++i)
    {
      const LocalMatrixFitPCL::Data & sample = samples[i];
      eigen_samples.row(i) << sample.x_, sample.y_, sample.weight_;
    }

    return eigen_samples;
  }

//  Eigen::Matrix<Scalar, Eigen::Dynamic, 3> getGlobalSamples(Size1 x_index,
//                                                            Size1 y_index) const
//  {
//    const std::vector<std::pair<Scalar, Scalar> > & samples = global_fit_->getSamples(x_index, y_index);
//    Eigen::Matrix<Scalar, Eigen::Dynamic, 3> eigen_samples(samples.size(), 3);

//    for (Size1 i = 0; i < samples.size(); ++i)
//      eigen_samples.row(i) << samples[i].first, samples[i].second, samples[i].first - samples[i].second;

//    return eigen_samples;
//  }

  void setDepthErrorFunction(const Polynomial<Scalar, 2> & depth_error_function)
  {
    depth_error_function_ = depth_error_function;
  }

private:

  bool extractPlane(const Checkerboard & color_cb,
                    const PCLCloud3::ConstPtr & cloud,
                    const Point3 & color_cb_center,
                    PlaneInfo & plane_info);

  Size1 max_threads_;

  LocalModel::Ptr local_model_;
  LocalMatrixFitPCL::Ptr local_fit_;

  GlobalModel::Ptr global_model_;
  GlobalMatrixFitPCL::Ptr global_fit_;

  InverseGlobalModel::Ptr inverse_global_model_;
  InverseGlobalMatrixFitEigen::Ptr inverse_global_fit_;

  std::vector<DepthData::Ptr> data_vec_;

  std::map<DepthData::ConstPtr, PlaneInfo> plane_info_map_;
  Polynomial<Scalar, 2> depth_error_function_;

};

} /* namespace calibration */
#endif /* RGBD_CALIBRATION_DEPTH_UNDISTORTION_ESTIMATION_H_ */
