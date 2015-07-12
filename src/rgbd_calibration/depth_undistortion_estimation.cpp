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

#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>

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
  Scalar radius = std::min(color_cb.width(), color_cb.height()) / 1.5; // TODO Add parameter
  PointPlaneExtraction<PCLPoint3> plane_extractor;
  plane_extractor.setInputCloud(cloud);
  plane_extractor.setRadius(radius);

  bool plane_extracted = false;

  int r[] = {0, 1, 2}; // TODO Add parameter
  int k[] = {0, 1, -1, 2, -2}; // TODO Add parameter
  for (Size1 i = 0; i < 5 and not plane_extracted; ++i)
  {
    for (Size1 j = 0; j < 3 and not plane_extracted; ++j)
    {
      plane_extractor.setRadius((1 + r[j]) * radius);
      plane_extractor.setPoint(PCLPoint3(center.x(), center.y(), center.z() + (1 + r[j]) * radius * k[i]));
      plane_extracted = plane_extractor.extract(plane_info);
    }
  }

  return plane_extracted;
}

void DepthUndistortionEstimation::estimateLocalModel()
{
  std::sort(data_vec_.begin(), data_vec_.end(), OrderByDistance());

  for (Size1 i = 0; i < data_vec_.size(); i += max_threads_)
  {
#pragma omp parallel for schedule(static, 1)
    for (Size1 th = 0; th < max_threads_; ++th)
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

//      RGBD_INFO(data.id_, "Transformed z: " << gt_cb.center().z() << " -> " << und_color_cb_center.z());

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
//        RGBD_INFO(data.id_, "Plane extracted!!");
        plane_info_map_[data_vec_[i + th]] = plane_info;

        if (i + th == 73 or i + th == 80)
        {
          PCLCloud3::Ptr tmp_und_cloud = boost::make_shared<PCLCloud3>(*und_cloud, *plane_info.indices_);
          PCLCloud3::Ptr tmp_cloud = boost::make_shared<PCLCloud3>(cloud, *plane_info.indices_);

          std::stringstream ss;
          pcl::PCDWriter writer;
          ss <<  "/tmp/tmp_und_cloud_" << i + th <<  ".pcd";
          writer.write(ss.str(), *und_cloud);

          ss.str("");
          ss <<  "/tmp/tmp_und_cloud_" << i + th <<  "_plane.pcd";
          writer.write(ss.str(), *tmp_und_cloud);

          ss.str("");
          ss <<  "/tmp/tmp_cloud_" << i + th <<  ".pcd";
          writer.write(ss.str(), cloud);

          ss.str("");
          ss <<  "/tmp/tmp_cloud_" << i + th <<  "_plane.pcd";
          writer.write(ss.str(), *tmp_cloud);

          PlaneInfo tmp_plane_info;
          if (extractPlane(gt_cb, data.cloud_, und_color_cb_center, tmp_plane_info))
          {
            PCLCloud3::Ptr tmp_cloud_2 = boost::make_shared<PCLCloud3>(cloud, *tmp_plane_info.indices_);

            ss.str("");
            ss <<  "/tmp/tmp_cloud_" << i + th <<  "_plane_2.pcd";
            writer.write(ss.str(), *tmp_cloud_2);
            std::cout << plane_info.std_dev_ << " -- " << tmp_plane_info.std_dev_ << std::endl;
          }
        }

//        Plane fitted_plane = PlaneFit<Scalar>::robustFit(PCLConversion<Scalar>::toPointMatrix(*und_cloud, *plane_info.indices_),
//                                                         plane_info.std_dev_);

        std::vector<int> indices;// = *plane_info.indices_;
        indices.reserve(plane_info.indices_->size());
        int w = und_cloud->width;
        int h = und_cloud->height;
        for (size_t j = 0; j < plane_info.indices_->size(); ++j)
        {
          int r = (*plane_info.indices_)[j] / w;
          int c = (*plane_info.indices_)[j] % w;
          if ((r - h/2)*(r - h/2) + (c - w/2)*(c - w/2) < (h/3)*(h/3))
            indices.push_back((*plane_info.indices_)[j]);
        }
        Plane fitted_plane = PlaneFit<Scalar>::fit(PCLConversion<Scalar>::toPointMatrix(cloud, indices)/*, plane_info.std_dev_*/);

        plane_info_map_[data_vec_[i + th]].plane_ = fitted_plane;


#pragma omp critical
        {
          local_fit_->accumulateCloud(cloud, *plane_info.indices_);
          local_fit_->addAccumulatedPoints(fitted_plane);
          for (Size1 c = 0; c < gt_cb.corners().elements(); ++c)
          {
            const Point3 & corner = gt_cb.corners()[c];
            inverse_global_fit_->addPoint(0, 0, corner, fitted_plane);
          }
          if (i + th > 20)
            inverse_global_fit_->update();

        }

        Line line(gt_cb.center(), Point3::UnitZ());
        RGBD_INFO(data.id_, "Transformed z: " << gt_cb.center().z() << " -> " << und_color_cb_center.z()
                                              << " (Real z: " << line.intersectionPoint(fitted_plane).z() << ")");

//      Scalar angle = RAD2DEG(std::acos(plane_info.equation_.normal().dot(gt_cb.plane().normal())));
//      RGBD_INFO(data.id(), "Angle: " << angle);

      }
      else
        RGBD_WARN(data.id_, "Plane not extracted!!");

    }

    local_fit_->update();
  }

}

void DepthUndistortionEstimation::estimateLocalModelReverse()
{
  //std::reverse(data_vec_.begin(), data_vec_.end());

  local_fit_->reset();

  for (Size1 i = 0; i < data_vec_.size(); i += max_threads_)
  {
#pragma omp parallel for schedule(static, 1)
    for (Size1 th = 0; th < max_threads_; ++th)
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

//      RGBD_INFO(data.id_, "Transformed z: " << gt_cb.center().z() << " -> " << und_color_cb_center.z());

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
//        RGBD_INFO(data.id_, "Plane extracted!!");

//        Plane fitted_plane = PlaneFit<Scalar>::robustFit(PCLConversion<Scalar>::toPointMatrix(*und_cloud, *plane_info.indices_),
//                                                         plane_info.std_dev_);

        boost::shared_ptr<std::vector<int> > indices = boost::make_shared<std::vector<int> >();// = *plane_info.indices_;
        indices->reserve(plane_info.indices_->size());
        int w = und_cloud->width;
        int h = und_cloud->height;
        for (size_t j = 0; j < plane_info.indices_->size(); ++j)
        {
          int r = (*plane_info.indices_)[j] / w;
          int c = (*plane_info.indices_)[j] % w;
          if ((r - h/2)*(r - h/2) + (c - w/2)*(c - w/2) < (h/3)*(h/3))
            indices->push_back((*plane_info.indices_)[j]);
        }
        Plane fitted_plane = PlaneFit<Scalar>::fit(PCLConversion<Scalar>::toPointMatrix(cloud, *indices)/*, plane_info.std_dev_*/);


        boost::shared_ptr<std::vector<int> > old_indices;
#pragma omp critical
        {
           old_indices = plane_info_map_[data_vec_[i + th]].indices_;
        }
        indices->clear();
        std::set_union(old_indices->begin(), old_indices->end(), plane_info.indices_->begin(), plane_info.indices_->end(), std::back_inserter(*indices));

#pragma omp critical
        {
          local_fit_->accumulateCloud(cloud, *indices);
          local_fit_->addAccumulatedPoints(fitted_plane);
          plane_info_map_[data_vec_[i + th]].indices_ = indices;
        }

        Line line(gt_cb.center(), Point3::UnitZ());
        RGBD_INFO(data.id_, "Transformed z: " << gt_cb.center().z() << " -> " << und_color_cb_center.z()
                                              << " (Real z: " << line.intersectionPoint(fitted_plane).z() << ")");

      }
      else
        RGBD_WARN(data.id_, "Plane not extracted!!");

    }

  }
  local_fit_->update();
  //std::reverse(data_vec_.begin(), data_vec_.end());

}

class LocalModelError
{
public:

  LocalModelError(const PCLCloud3::ConstPtr & cloud,
                  const Plane & plane,
                  const LocalModel::Ptr & local_model,
                  const Polynomial<double, 2> & depth_error_function)
    : cloud_(cloud), plane_(plane), depth_error_function_(depth_error_function)
  {
    local_matrix_.setModel(local_model);
  }

  void addIndex(int x_index, int y_index)
  {
    indices_.push_back(std::make_pair(x_index, y_index));
  }

  void createCloudWithIndices()
  {
    part_cloud_.height = 1;
    part_cloud_.width = indices_.size();
    part_cloud_.points.resize(indices_.size());
    for (Size1 i = 0; i < indices_.size(); ++i)
    {
      int index = indices_[i].first + cloud_->width * indices_[i].second;
      part_cloud_.points[i] = cloud_->points[index];
    }
  }

  int size() const
  {
    return indices_.size();
  }

  bool operator ()(const double * const local_poly_1_data,
                   const double * const local_poly_2_data,
                   const double * const local_poly_3_data,
                   const double * const local_poly_4_data,
                   double * residuals) const
  {
    PCLCloud3 tmp_cloud = part_cloud_;

    Eigen::Map<Eigen::Matrix3Xd> residuals_map(residuals, 3, size());
    Eigen::MatrixX4d polynomials(MathTraits<LocalPolynomial>::Size, 4);
    polynomials.col(0) = Eigen::Map<const Eigen::Matrix<double, MathTraits<LocalPolynomial>::Size, 1> >(local_poly_1_data);
    polynomials.col(1) = Eigen::Map<const Eigen::Matrix<double, MathTraits<LocalPolynomial>::Size, 1> >(local_poly_2_data);
    polynomials.col(2) = Eigen::Map<const Eigen::Matrix<double, MathTraits<LocalPolynomial>::Size, 1> >(local_poly_3_data);
    polynomials.col(3) = Eigen::Map<const Eigen::Matrix<double, MathTraits<LocalPolynomial>::Size, 1> >(local_poly_4_data);

    for (Size1 i = 0; i < indices_.size(); ++i)
    {
      const std::pair<int, int> & index = indices_[i];

      std::vector<LocalModel::LookupTableData> lt_data = local_matrix_.model()->lookupTable(index.first, index.second);

      double tmp_depth = 0.0;
      double depth = tmp_cloud.points[i].z;
      for (Size1 j = 0; j < lt_data.size(); ++j)
        tmp_depth += lt_data[j].weight_ * LocalPolynomial::evaluate(polynomials.col(j), depth);
      depth = tmp_depth;
      double k = depth / tmp_cloud.points[i].z;

      Point3 p(tmp_cloud.points[i].x * k, tmp_cloud.points[i].y * k, depth);
      Line line(Vector3::Zero(), p.normalized());
//      residuals[i] = (p - line.intersectionPoint(plane_)).norm() / ceres::poly_eval(depth_error_function_.coefficients(), p.z());
      residuals_map.col(i) = (p - line.intersectionPoint(plane_)) / ceres::poly_eval(depth_error_function_.coefficients(), p.z());
    }

    return true;
  }

private:

  const PCLCloud3::ConstPtr cloud_;
  const Plane plane_;
  LocalMatrixPCL local_matrix_;
  const Polynomial<double, 2> depth_error_function_;
  std::vector<std::pair<int, int> > indices_;
  PCLCloud3 part_cloud_;

};

typedef ceres::NumericDiffCostFunction<LocalModelError, ceres::CENTRAL, ceres::DYNAMIC, MathTraits<LocalPolynomial>::Size,
MathTraits<LocalPolynomial>::Size, MathTraits<LocalPolynomial>::Size, MathTraits<LocalPolynomial>::Size> LocalCostFunction;

void DepthUndistortionEstimation::optimizeLocalModel(const Polynomial<double, 2> & depth_error_function)
{
  std::vector<LocalModelError *> all_errors_vec;
  ceres::Problem problem;

  for (Size1 i = 0; i < data_vec_.size(); ++i)
  {
    const DepthData::ConstPtr & data = data_vec_[i];
    const std::vector<int> & indices = *plane_info_map_[data].indices_;

    int delta_x = data->cloud_->width / local_model_->binSize().x();
    int delta_y = data->cloud_->height / local_model_->binSize().y();

    std::vector<LocalModelError *> error_vec;

    for (Size1 j = 0; j < delta_y * delta_x; ++j)
    {
      error_vec.push_back(new LocalModelError(data->cloud_, plane_info_map_[data].plane_, local_model_, depth_error_function));
      all_errors_vec.push_back(error_vec.back());
    }

    for (Size1 j = 0; j < indices.size(); ++j)
    {
      int x_index = indices[j] % data->cloud_->width;
      int y_index = indices[j] / data->cloud_->width;
      int bin_x = x_index / local_model_->binSize().x();
      int bin_y = y_index / local_model_->binSize().y();
      error_vec[bin_x + delta_x * bin_y]->addIndex(x_index, y_index);
    }

    for (Size1 j = 0; j < error_vec.size(); ++j)
    {
      if (error_vec[j]->size() == 0)
        continue;
      error_vec[j]->createCloudWithIndices();

      int x_index = j % delta_x;
      int y_index = j / delta_x;

//      ceres::CostFunction * cost_function = new LocalCostFunction(error_vec[j], ceres::DO_NOT_TAKE_OWNERSHIP, error_vec[j]->size());
      ceres::CostFunction * cost_function = new LocalCostFunction(error_vec[j], ceres::DO_NOT_TAKE_OWNERSHIP, 3 * error_vec[j]->size());
      problem.AddResidualBlock(cost_function, NULL,
                               local_model_->matrix()->at(x_index, y_index).data(),
                               local_model_->matrix()->at(x_index, y_index + 1).data(),
                               local_model_->matrix()->at(x_index + 1, y_index).data(),
                               local_model_->matrix()->at(x_index + 1, y_index + 1).data());

    }

  }


  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.max_num_iterations = 10;
  options.minimizer_progress_to_stdout = true;
  options.num_threads = 8;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  for (Size1 i = 0; i < all_errors_vec.size(); ++i)
    delete all_errors_vec[i];

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

//    RGBD_INFO(data.id_, " - Transformed z: " << gt_cb.center().z() << " -> " << und_color_cb_center.z());

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
        Indices reduced = *plane_info.indices_;
        std::random_shuffle(reduced.begin(), reduced.end());
        reduced.resize(reduced.size() / 5);
        global_fit_->accumulateCloud(*und_cloud, reduced);
        global_fit_->addAccumulatedPoints(gt_cb.plane());
      }
    }
    else
      RGBD_WARN(data.id_, "Plane not extracted!!");

  }
  global_fit_->update();

}

} /* namespace calibration */
