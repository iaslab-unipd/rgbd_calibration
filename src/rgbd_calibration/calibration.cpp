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
#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/random_sample.h>

#include <calibration_common/ceres/plane_fit.h>
#include <kinect/depth/polynomial_matrix_io.h>

#include <rgbd_calibration/checkerboard_views_extractor.h>
#include <rgbd_calibration/plane_based_extrinsic_calibration.h>
#include <rgbd_calibration/depth_undistortion_estimation.h>

#include <rgbd_calibration/calibration.h>

#define RGBD_INFO(id, msg) ROS_INFO_STREAM("RGBD " << id << ": " << msg)

namespace calibration
{

class CheckerboardDistanceConstraint : public Constraint<Checkerboard>
{

public:

  CheckerboardDistanceConstraint(Scalar distance,
                                 const Point3 & from = Point3::Zero())
    : distance_(distance),
      from_(from)
  {
    // Do nothing
  }

  virtual ~CheckerboardDistanceConstraint()
  {
    // Do nothing
  }

  inline virtual bool isValid(const Checkerboard & checkerboard) const
  {
    return (checkerboard.center() - from_).norm() <= distance_;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:

  Scalar distance_;
  Point3 from_;

};

void Calibration::publishData() const
{
  if (not publisher_)
    return;

  publisher_->publishTF(*depth_sensor_);
  publisher_->publishTF(*color_sensor_);

  for (size_t i = 0; i < test_vec_.size(); i += 1)
    publisher_->publish(*test_vec_[i]);
  //
  //  for (size_t i = 0; i < rgbd_cb_vec_.size(); i += 1)
  //    publisher_->publish(*rgbd_cb_vec_[i]);
  //
  //  for (size_t i = 0; i < und_rgbd_cb_vec_.size(); i += 1)
  //    publisher_->publish(*und_rgbd_cb_vec_[i], "und_");

}

void Calibration::addData(const cv::Mat & image,
                          const PCLCloud3::ConstPtr & cloud)
{
//  PCLCloud3::Ptr new_cloud(boost::make_shared<PCLCloud3>());
//  std::vector<int> remapping;
//  pcl::removeNaNFromPointCloud(*cloud, *new_cloud, remapping);

//  if (ratio_ > 1)
//  {
//    pcl::RandomSample<PCLPoint3> random_sample;
//    random_sample.setInputCloud(new_cloud);
//    random_sample.setSample(new_cloud->size() / ratio_);
//    random_sample.setSeed(rand());
//    random_sample.filter(*new_cloud);
//  }

  RGBDData::Ptr data(boost::make_shared<RGBDData>(data_vec_.size() + 1));
  data->setColorSensor(color_sensor_);
  data->setDepthSensor(depth_sensor_);
  data->setColorData(image);
//  data->setDepthData(*new_cloud);
  data->setDepthData(*cloud);

  data_vec_.push_back(data);
}

void Calibration::addTestData(const cv::Mat & image,
                              const PCLCloud3::ConstPtr & cloud)
{
//  PCLCloud3::Ptr new_cloud(boost::make_shared<PCLCloud3>());
//  std::vector<int> remapping;
//  pcl::removeNaNFromPointCloud(*cloud, *new_cloud, remapping);

//  if (ratio_ > 1)
//  {
//    pcl::RandomSample<PCLPoint3> random_sample;
//    random_sample.setInputCloud(new_cloud);
//    random_sample.setSample(new_cloud->size() / ratio_);
//    random_sample.setSeed(rand());
//    random_sample.filter(*new_cloud);
//  }

  RGBDData::Ptr data(boost::make_shared<RGBDData>(data_vec_.size() + 1));
  data->setColorSensor(color_sensor_);
  data->setDepthSensor(depth_sensor_);
  data->setColorData(image);
//  data->setDepthData(*new_cloud);
  data->setDepthData(*cloud);

  test_vec_.push_back(data);
}

void Calibration::perform()
{
  if (estimate_initial_trasform_ or not color_sensor_->parent())
    estimateInitialTransform();

  if (estimate_depth_und_model_)
  {
    CheckerboardViewsExtraction cb_extractor;
    cb_extractor.setColorSensorPose(color_sensor_->pose());
    cb_extractor.setCheckerboardVector(cb_vec_);
    cb_extractor.setInputData(data_vec_);
    cb_extractor.setOnlyImages(true);
    cb_extractor.extractAll(cb_views_vec_);

    for (size_t i = 0; i < cb_views_vec_.size(); ++i)
    {
      CheckerboardViews & cb_views = *cb_views_vec_[i];
      Checkerboard::Ptr cb = boost::make_shared<Checkerboard>(*cb_views.colorCheckerboard());
      cb->transform(color_sensor_->pose());
      depth_data_vec_.push_back(depth_undistortion_estimation_->addDepthData(cb_views.data()->depthData(), cb));
    }

    depth_undistortion_estimation_->estimateLocalModel();
    depth_undistortion_estimation_->estimateGlobalModel();

    for (size_t i = 0; i < cb_views_vec_.size(); ++i)
    {
      CheckerboardViews & cb_views = *cb_views_vec_[i];
      cb_views.setPlaneInliers(depth_data_vec_[i]->estimated_plane_);
    }

//    PolynomialUndistortionMatrixIO<LocalPolynomial> io;
//    io.write(*local_und_->model(), "/tmp/local_matrix.txt");

//    Scalar max;
//    io.writeImageAuto(*local_und_->model(), Scalar(4.0), "/tmp/matrix4.png", max);
//    ROS_INFO_STREAM("Max 4: " << max);
//    io.writeImageAuto(*local_und_->model(), Scalar(3.0), "/tmp/matrix3.png", max);
//    ROS_INFO_STREAM("Max 3: " << max);
//    io.writeImageAuto(*local_und_->model(), Scalar(2.0), "/tmp/matrix2.png", max);
//    ROS_INFO_STREAM("Max 2: " << max);
//    io.writeImageAuto(*local_und_->model(), Scalar(1.0), "/tmp/matrix1.png", max);
//    ROS_INFO_STREAM("Max 1: " << max);

//    PolynomialUndistortionMatrixIO<GlobalPolynomial> io2;
//    io2.write(*global_und_->model(), "/tmp/global_matrix.txt");

//    int bins[] = {10, 20, 30, 40, 50};

//    for (int i = 0; i < 10; ++i)
//    {
//      int n = 5 * (i + 1);
//      std::cout << "L Polynomial at (" << n << ", " << n << "): " << local_und_->model()->polynomial(n, n).transpose()
//                << std::endl;
//      std::stringstream ss;
//      ss << "/tmp/l_samples_" << n << "_" << n << ".txt";
//      std::fstream fs;
//      fs.open(ss.str().c_str(), std::fstream::out);
//      fs << depth_undistortion_estimation_->getLocalSamples(n, n) << std::endl;
//      fs.close();
//    }

//    for (int i = 0; i < 4; ++i)
//    {
//      std::cout << "G Polynomial at (" << i/2 << ", " << i%2 << "): " << global_und_->model()->polynomial(i/2, i%2).transpose()
//                << std::endl;

//      std::stringstream ss;
//      ss << "/tmp/g_samples_" << i/2 << "_" << i%2 << ".txt";

//      std::fstream fs;
//      fs.open(ss.str().c_str(), std::fstream::out);
//      fs << depth_undistortion_estimation_->getGlobalSamples(i/2, i%2) << std::endl;
//      fs.close();
//    }

  }

  estimateTransform(cb_views_vec_);

}

void Calibration::estimateInitialTransform()
{
  CheckerboardViewsExtraction cb_extractor;
  cb_extractor.setCheckerboardVector(cb_vec_);
  cb_extractor.setCheckerboardConstraint(boost::make_shared<CheckerboardDistanceConstraint>(2.0));

  std::vector<CheckerboardViews::Ptr> cb_views_vec;

  for (size_t i = 0; i < data_vec_.size() and cb_views_vec.size() < 10; ++i)
  {
    size_t index = rand() % data_vec_.size();
    cb_extractor.setInputData(data_vec_[index]);
    cb_extractor.extract(cb_views_vec, true);
  }

  estimateTransform(cb_views_vec);
}

void Calibration::estimateTransform(const std::vector<CheckerboardViews::Ptr> & cb_views_vec)
{
  PlaneBasedExtrinsicCalibration calib;
  calib.setMainSensor(depth_sensor_);
  calib.setSize(cb_views_vec.size());

  for (size_t i = 0; i < cb_views_vec.size(); ++i)
  {
    calib.addData(i, color_sensor_, cb_views_vec[i]->colorCheckerboard());
    calib.addData(i, depth_sensor_, cb_views_vec[i]->depthPlane());
  }

  calib.perform();
}

class TransformError
{
public:

  TransformError(const PinholeCameraModel::ConstPtr & camera_model,
                 const Checkerboard::ConstPtr & checkerboard,
                 const Cloud2 & image_corners,
                 const Plane & depth_plane,
                 const Polynomial<Scalar, 2> & depth_error_function)
    : camera_model_(camera_model),
      checkerboard_(checkerboard),
      image_corners_(image_corners),
      depth_plane_(depth_plane),
      depth_error_function_(depth_error_function)
  {
  }

  template <typename T>
  bool operator ()(const T * const color_sensor_pose,
                   const T * const checkerboard_pose,
                   T * residuals) const
  {
    typename Types<T>::Vector3 color_sensor_r_vec(color_sensor_pose[0], color_sensor_pose[1], color_sensor_pose[2]);
    typename Types<T>::AngleAxis color_sensor_r(color_sensor_r_vec.norm(), color_sensor_r_vec.normalized());
    typename Types<T>::Translation3 color_sensor_t(color_sensor_pose[3], color_sensor_pose[4], color_sensor_pose[5]);

    typename Types<T>::Transform color_sensor_pose_eigen = color_sensor_t * color_sensor_r;

    typename Types<T>::Vector3 checkerboard_r_vec(checkerboard_pose[0], checkerboard_pose[1], checkerboard_pose[2]);
    typename Types<T>::AngleAxis checkerboard_r(checkerboard_r_vec.norm(), checkerboard_r_vec.normalized());
    typename Types<T>::Translation3 checkerboard_t(checkerboard_pose[3], checkerboard_pose[4], checkerboard_pose[5]);

    typename Types<T>::Transform checkerboard_pose_eigen = checkerboard_t * checkerboard_r;

    typename Types<T>::Cloud3 cb_corners(checkerboard_->corners().size());
    cb_corners.container() = checkerboard_pose_eigen * checkerboard_->corners().container().cast<T>();

    typename Types<T>::Plane depth_plane(depth_plane_.normal().cast<T>(), T(depth_plane_.offset()));
    typename Types<T>::Cloud2 reprojected_corners = camera_model_->project3dToPixel<T>(cb_corners);

    cb_corners.transform(color_sensor_pose_eigen);

    Polynomial<T, 2> depth_error_function(depth_error_function_.coefficients().cast<T>());

    for (Size1 i = 0; i < cb_corners.elements(); ++i)
    {
      residuals[2 * i] = T((reprojected_corners[i] - image_corners_[i].cast<T>()).norm() / 0.5);
      residuals[2 * i + 1] = T(depth_plane.absDistance(cb_corners[i])
                               / ceres::poly_eval(depth_error_function.coefficients(), cb_corners[i].z())); // TODO use line-of-sight error
    }

    return true;
  }

private:

  const PinholeCameraModel::ConstPtr & camera_model_;
  const Checkerboard::ConstPtr & checkerboard_;
  const Cloud2 & image_corners_;
  const Plane & depth_plane_;
  const Polynomial<Scalar, 2> depth_error_function_;

};

void Calibration::optimizeTransform(const std::vector<CheckerboardViews::Ptr> & cb_views_vec)
{
  ceres::Problem problem;
  Eigen::Matrix<Scalar, Eigen::Dynamic, 6, Eigen::DontAlign | Eigen::RowMajor> data(cb_views_vec.size(), 6);

  AngleAxis rotation(color_sensor_->pose().linear());
  Translation3 translation(color_sensor_->pose().translation());

  Eigen::Matrix<Scalar, 1, 6, Eigen::DontAlign | Eigen::RowMajor> transform;
  transform.head<3>() = rotation.angle() * rotation.axis();
  transform.tail<3>() = translation.vector();

  for (size_t i = 0; i < cb_views_vec.size(); ++i)
  {
    const CheckerboardViews & cb_views = *cb_views_vec[i];

    rotation = AngleAxis(cb_views.colorCheckerboard()->pose().linear());
    data.row(i).head<3>() = rotation.angle() * rotation.axis();
    data.row(i).tail<3>() = cb_views.colorCheckerboard()->pose().translation();

    TransformError * error = new TransformError(color_sensor_->cameraModel(),
                                                cb_views.checkerboard(),
                                                cb_views.colorView()->points(),
                                                cb_views.depthPlane()->plane(),
                                                depth_sensor_->depthErrorFunction());

    typedef ceres::AutoDiffCostFunction<TransformError, ceres::DYNAMIC, 6, 6> TransformCostFunction;

    ceres::CostFunction * cost_function = new TransformCostFunction(error, 2 * cb_views.checkerboard()->size());
    problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(1.0), transform.data(), data.row(i).data());
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_SCHUR;
  options.max_num_iterations = 100;
  options.minimizer_progress_to_stdout = true;
  options.num_threads = 8;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  rotation.angle() = transform.head<3>().norm();
  rotation.axis() = transform.head<3>().normalized();
  translation.vector() = transform.tail<3>();

  color_sensor_->setPose(translation * rotation);

}

class TransformDistortionError
{
public:

  TransformDistortionError(const PinholeCameraModel::ConstPtr & camera_model,
                           const Checkerboard::ConstPtr & checkerboard,
                           const Cloud2 & image_corners,
                           const Cloud3 & depth_plane_points,
                           const Polynomial<Scalar, 2> & depth_error_function,
                           const Size2 & images_size)
    : camera_model_(camera_model),
      checkerboard_(checkerboard),
      image_corners_(image_corners),
      depth_plane_points_(depth_plane_points),
      depth_error_function_(depth_error_function),
      images_size_(images_size)
  {

//    Size1 c = 0;
//    for (int i = 0; i < depth_plane_points_.elements(); ++i)
//      if (depth_plane_points_[i].hasNaN())
//        c++;

//    if (c > 0)
//      ROS_INFO_STREAM("NaN = " << c);

  }

  template <typename T>
    typename Types<T>::Pose toEigen(const T * const pose) const
    {
      typename Types<T>::Vector3 rot_vec(pose[0], pose[1], pose[2]);
      typename Types<T>::AngleAxis rotation(rot_vec.norm(), rot_vec.normalized());
      typename Types<T>::Translation3 translation(pose[3], pose[4], pose[5]);
      return translation * rotation;
    }

  template <typename T>
    bool operator ()(const T * const color_sensor_pose,
                     const T * const global_undistortion,
                     const T * const checkerboard_pose,
                     T * residuals) const
    {
      typename Types<T>::Pose color_sensor_pose_eigen = toEigen<T>(color_sensor_pose);
      typename Types<T>::Pose checkerboard_pose_eigen = toEigen<T>(checkerboard_pose);

      GlobalModel::Data::Ptr global_data = boost::make_shared<GlobalModel::Data>(Size2(2, 2));
      for (int i = 0; i < 4 * MathTraits<GlobalPolynomial>::Size; ++i)
        global_data->container().data()[i] = global_undistortion[i];

      GlobalModel::Ptr global_model = boost::make_shared<GlobalModel>(images_size_);
      global_model->setMatrix(global_data);

      GlobalMatrixEigen global(global_model);

      typename Types<T>::Cloud3 depth_plane_points(depth_plane_points_.size());
      depth_plane_points.container() = depth_plane_points_.container().cast<T>();

      Size1 c = 0;
      for (int i = 0; i < depth_plane_points.elements(); ++i)
        if (depth_plane_points[i].hasNaN())
          c++;

      if (c > 0)
        ROS_INFO_STREAM("NaN = " << c);

      global.undistort(depth_plane_points);

      Size1 c2 = 0;
      for (int i = 0; i < depth_plane_points.elements(); ++i)
        if (depth_plane_points[i].hasNaN())
          c2++;

      if (c2 > 0)
        ROS_INFO_STREAM("NaN = " << c << " -> " << c2);


      typename Types<T>::Plane depth_plane(PlaneFit<T>::fit(depth_plane_points));

      typename Types<T>::Point3 z00(T(-1.0), T(-1.0), T(2.0));
      typename Types<T>::Point3 z01(T(-1.0), T( 1.0), T(2.0));
      typename Types<T>::Point3 z10(T( 1.0), T(-1.0), T(2.0));
      typename Types<T>::Point3 z11(T( 1.0), T( 1.0), T(2.0));

      global.undistort(0, 0, z00);
      global.undistort(0, 1, z01);
      global.undistort(1, 0, z10);
      global.undistort(1, 1, z11);

//      T penalty;
//      for (size_t i = 0; i < depth_plane_points.size(); ++i)
//      {
//        Line line(Point3::Zero(), depth_plane_points[i]);
//        penalty += (line.intersectionPoint(depth_plane) - depth_plane_points[i]).squaredNorm();
//      }
//      penalty /= T(depth_plane_points.size());
//      penalty = std::exp(penalty);

      //typename Types<T>::Cloud3 cb_corners(checkerboard_->cols(), checkerboard_->rows());
      typename Types<T>::Cloud3 cb_corners(checkerboard_->corners().size());
      cb_corners.container() = checkerboard_pose_eigen * checkerboard_->corners().container().cast<T>();

      typename Types<T>::Cloud2 reprojected_corners = camera_model_->project3dToPixel<T>(cb_corners);

      cb_corners.transform(color_sensor_pose_eigen);

      Polynomial<T, 2> depth_error_function(depth_error_function_.coefficients().cast<T>());

      typename Types<T>::Plane penalty_plane = Types<T>::Plane::Through(z00, z01, z10);
      typename Types<T>::Line line(Types<T>::Point3::Zero(), z11);
      residuals[0] = (line.intersectionPoint(penalty_plane) - z11).norm() / ceres::poly_eval(depth_error_function.coefficients(), T(2.0));

      for (size_t i = 0; i < cb_corners.size().prod(); ++i)
      {
        residuals[2 * i + 1] = T((reprojected_corners[i] - image_corners_[i].cast<T>()).norm() / 0.5);
//        residuals[2 * i + 1] = T(
//          depth_plane.absDistance(cb_corners[i]) / ceres::poly_eval(depth_error_function.coefficients(),
//                                                                    cb_corners[i].z())); // TODO use line-of-sight error
        Line line(Point3::Zero(), cb_corners[i]);
        residuals[2 * i + 2] = T((line.intersectionPoint(depth_plane) - cb_corners[i]).norm()
                                 / ceres::poly_eval(depth_error_function.coefficients(), cb_corners[i].z()));
      }

      return true;
    }

private:

  const PinholeCameraModel::ConstPtr & camera_model_;
  const Checkerboard::ConstPtr & checkerboard_;
  const Cloud2 & image_corners_;

  const Cloud3 & depth_plane_points_;

  const Polynomial<Scalar, 2> depth_error_function_;
  const Size2 & images_size_;

};

void Calibration::optimizeAll(const std::vector<CheckerboardViews::Ptr> & cb_views_vec)
{
  ceres::Problem problem;
  Eigen::Matrix<Scalar, Eigen::Dynamic, 6, Eigen::DontAlign | Eigen::RowMajor> data(cb_views_vec.size(), 6);

  AngleAxis rotation(color_sensor_->pose().linear());
  Translation3 translation(color_sensor_->pose().translation());

  Eigen::Matrix<Scalar, 1, 6, Eigen::DontAlign | Eigen::RowMajor> transform;
  transform.head<3>() = rotation.angle() * rotation.axis();
  transform.tail<3>() = translation.vector();

  for (size_t i = 0; i < cb_views_vec.size(); ++i)
  {
    const CheckerboardViews & cb_views = *cb_views_vec[i];

    rotation = AngleAxis(cb_views.colorCheckerboard()->pose().linear());
    data.row(i).head<3>() = rotation.angle() * rotation.axis();
    data.row(i).tail<3>() = cb_views.colorCheckerboard()->pose().translation();

    TransformDistortionError * error = new TransformDistortionError(color_sensor_->cameraModel(),
                                                                    cb_views.checkerboard(),
                                                                    cb_views.colorView()->points(),
                                                                    cb_views.depthView()->points(),
                                                                    depth_sensor_->depthErrorFunction(),
                                                                    global_model_->imageSize());

    typedef ceres::NumericDiffCostFunction<TransformDistortionError, ceres::CENTRAL, ceres::DYNAMIC, 6,
        4 * MathTraits<GlobalPolynomial>::Size, 6> TransformDistortionCostFunction;

    ceres::CostFunction * cost_function = new TransformDistortionCostFunction(error,
                                                                              ceres::DO_NOT_TAKE_OWNERSHIP,
                                                                              1 + 2 * cb_views.checkerboard()->size());
    problem.AddResidualBlock(cost_function,
                             new ceres::CauchyLoss(1.0),
                             transform.data(),
                             global_matrix_->model()->dataPtr(),
                             data.row(i).data());
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_SCHUR;
  options.max_num_iterations = 100;
  options.minimizer_progress_to_stdout = true;
  options.num_threads = 8;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  rotation.angle() = transform.head<3>().norm();
  rotation.axis() = transform.head<3>().normalized();
  translation.vector() = transform.tail<3>();

  color_sensor_->setPose(translation * rotation);
}

void Calibration::optimize()
{
  if (estimate_depth_und_model_)
  {
    std::vector<CheckerboardViews::Ptr> und_cb_views_vec;

    // Create locally undistorted clouds and views
#pragma omp parallel for
    for (size_t i = 0; i < cb_views_vec_.size(); ++i)
    {
      const CheckerboardViews & cb_views = *cb_views_vec_[i];
      const DepthUndistortionEstimation::DepthData & depth_data = *depth_data_vec_[i];

      if (not depth_data.plane_extracted_)
        continue;

      CheckerboardViews::Ptr und_cb_views = boost::make_shared<CheckerboardViews>(cb_views);

      RGBDData::Ptr und_data = boost::make_shared<RGBDData>(*cb_views.data());
      und_data->setDepthData(*depth_data.undistorted_cloud_);

      und_cb_views->setId(cb_views.id() + "_undistorted");
      und_cb_views->setData(und_data);
      und_cb_views->setPlaneInliers(depth_data.estimated_plane_.indices_, depth_data.estimated_plane_.std_dev_);

#pragma omp critical
      und_cb_views_vec.push_back(und_cb_views);
    }

    optimizeAll(und_cb_views_vec);
  }
  else
    optimizeTransform(cb_views_vec_);

  PolynomialUndistortionMatrixIO<GlobalPolynomial> io;
  io.write(*global_matrix_->model(), "/tmp/opt_global_matrix.txt");

}

} /* namespace calibration */
