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

#include <rgbd_calibration/calibration_test.h>
#include <rgbd_calibration/checkerboard_views_extractor.h>

#include <calibration_common/base/pcl_conversion.h>

#include <pcl/filters/random_sample.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include <ceres/ceres.h>

namespace calibration
{

void CalibrationTest::publishData() const
{
  if (not publisher_)
    return;

  publisher_->publishTF(*depth_sensor_);
  publisher_->publishTF(*color_sensor_);

  for (size_t i = 0; i < data_vec_.size(); i += 1)
    publisher_->publish(*data_vec_[i]);

}

void CalibrationTest::addData(const cv::Mat & image,
                              const PCLCloud3::ConstPtr & cloud)
{
//  PCLCloud3::Ptr new_cloud = boost::make_shared<PCLCloud3>();
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

  int index = data_vec_.size() + 1;

  cv::Mat rectified;
  color_sensor_->cameraModel()->rectifyImage(image, rectified);

  RGBDData::Ptr data(boost::make_shared<RGBDData>(index));
  data->setColorSensor(color_sensor_);
  data->setDepthSensor(depth_sensor_);
  data->setColorData(rectified);
  data->setDepthData(*cloud);

  PCLCloud3::Ptr part_und_cloud = boost::make_shared<PCLCloud3>(*cloud);
  local_matrix_->undistort(*part_und_cloud);

  RGBDData::Ptr part_und_data(boost::make_shared<RGBDData>(index));
  part_und_data->setColorSensor(color_sensor_);
  part_und_data->setDepthSensor(depth_sensor_);
  part_und_data->setColorData(image);
  part_und_data->setDepthData(*part_und_cloud);

  PCLCloud3::Ptr und_cloud = boost::make_shared<PCLCloud3>(*part_und_cloud);
  global_matrix_->undistort(*und_cloud);

  RGBDData::Ptr und_data(boost::make_shared<RGBDData>(index));
  und_data->setColorSensor(color_sensor_);
  und_data->setDepthSensor(depth_sensor_);
  und_data->setColorData(image);
  und_data->setDepthData(*und_cloud);

  data_vec_.push_back(data);
  part_und_data_vec_.push_back(part_und_data);
  und_data_vec_.push_back(und_data);

  part_data_map_[und_data] = part_und_data;
  data_map_[und_data] = data;

  //  float z = 0.0f;
  //  for (size_t i = 0; i < und_cloud->size(); ++i)
  //    z += und_cloud->points[i].z;
  //  z /= und_cloud->size();

  //  PCLCloud::Ptr tmp_cloud = boost::make_shared<PCLCloud>();
  //  pcl::PassThrough<pcl::PointXYZ> pass;
  //  pass.setInputCloud(new_cloud);
  //  pass.setFilterFieldName("y");
  //  pass.setFilterLimits(-3.0f, 1.0f);
  //  pass.filter(*tmp_cloud);
  //
  //  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  //  voxel.setInputCloud(tmp_cloud);
  //  voxel.setLeafSize(0.05f, 10.0f, 0.05f);
  //  voxel.filter(*tmp_cloud);
  //
  //  std::fstream fs;
  //  fs.open("/tmp/points.txt", std::fstream::out | std::fstream::app);
  //  for (size_t i = 0; i < tmp_cloud->size(); ++i)
  //    fs << tmp_cloud->points[i].x << " " << tmp_cloud->points[i].z << " " << tmp_cloud->points[i].y << std::endl;
  //  fs << "------------------------------------------------------------------" << std::endl;
  //  fs.close();
  //
  //  pass.setInputCloud(und_cloud);
  //  pass.filter(*tmp_cloud);
  //  voxel.setInputCloud(tmp_cloud);
  //  voxel.filter(*tmp_cloud);
  //
  //  fs.open("/tmp/und_points.txt", std::fstream::out | std::fstream::app);
  //  for (size_t i = 0; i < tmp_cloud->size(); ++i)
  //    fs << tmp_cloud->points[i].x << " " << tmp_cloud->points[i].z << " " << tmp_cloud->points[i].y << std::endl;
  //  fs << "------------------------------------------------------------------" << std::endl;
  //  fs.close();

}

void CalibrationTest::testPlanarityError() const
{
  std::vector<CheckerboardViews::Ptr> cb_views_vec;

  CheckerboardViewsExtraction cb_extractor;
  cb_extractor.setColorSensorPose(color_sensor_->pose());
  cb_extractor.setCheckerboardVector(cb_vec_);
  cb_extractor.setInputData(und_data_vec_);
  cb_extractor.extractAll(cb_views_vec);

  for (size_t i = 0; i < cb_views_vec.size(); ++i)
  {
    const CheckerboardViews & cb_views = *cb_views_vec[i];
    RGBDData::ConstPtr und_data = cb_views.data();
    RGBDData::ConstPtr data = data_map_.at(und_data);

    const Cloud3 & und_points = PCLConversion<Scalar>::toPointMatrix(*cb_views.depthView()->data(), cb_views.depthView()->points());
    Plane und_plane = PlaneFit<Scalar>::fit(und_points);

    Scalar d_mean = 0;
    Scalar und_mean = 0;
    Scalar und_max = 0;
    Scalar und_var = 0;
    for (int p = 0; p < und_points.elements(); ++p)
    {
      d_mean += und_points[p].z();
      Scalar d = und_plane.absDistance(und_points[p]);
      und_mean += d;
      und_var += d * d;
      if (d > und_max)
        und_max = d;
    }

    d_mean /= und_points.elements();
    und_mean /= und_points.elements();
    und_var /= und_points.elements();
    und_var -= und_mean * und_mean;

    const Cloud3 points = PCLConversion<Scalar>::toPointMatrix(*data->depthData(), *cb_views.planeInliers());
    Plane plane = PlaneFit<Scalar>::fit(points);

    Scalar mean = 0;
    Scalar max = 0;
    Scalar var = 0;
    for (int p = 0; p < points.elements(); ++p)
    {
      Scalar d = plane.absDistance(points[p]);
      mean += d;
      var += d * d;
      if (d > max)
        max = d;
    }

    mean /= points.elements();
    var /= points.elements();
    var -= mean * mean;

    std::cout << d_mean << " " << mean << " " << std::sqrt(var) << " " << max << " ** " << und_mean << " " << std::sqrt(und_var) << " " << und_max << std::endl;
  }

}

class NormalError
{
public:

  NormalError(const PinholeCameraModel::ConstPtr & camera_model,
              const Checkerboard::ConstPtr & checkerboard,
              const Cloud2 & image_corners)
    : camera_model_(camera_model),
      checkerboard_(checkerboard),
      image_corners_(image_corners)
  {
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
  bool operator ()(const T * const normal,
                   const T * const checkerboard_pose,
                   T * residuals) const
  {
    typename Types<T>::Pose checkerboard_pose_eigen = toEigen<T>(checkerboard_pose);

    typename Types<T>::Cloud3 cb_corners(checkerboard_->corners().size());
    cb_corners.container() = checkerboard_pose_eigen * checkerboard_->corners().container().cast<T>();
    typename Types<T>::Plane plane = Types<T>::Plane::Through(cb_corners(0, 0),
                                                              cb_corners(checkerboard_->cols() - 1, 0),
                                                              cb_corners(0, checkerboard_->rows() - 1));

    typename Types<T>::Cloud2 reprojected_corners = camera_model_->project3dToPixel<T>(cb_corners);
    typename Types<T>::Vector3 eigen_normal(normal[0], normal[1], normal[2]);
    eigen_normal.normalize();

    residuals[0] = ceres::acos(plane.normal().dot(eigen_normal));
    for (Size1 i = 0; i < cb_corners.elements(); ++i)
    {
      residuals[i + 1] = T((reprojected_corners[i] - image_corners_[i].cast<T>()).norm());
    }

    return true;
  }

private:

  const PinholeCameraModel::ConstPtr & camera_model_;
  const Checkerboard::ConstPtr & checkerboard_;
  const Cloud2 & image_corners_;

};

void CalibrationTest::testCheckerboardError() const
{
  std::vector<CheckerboardViews::Ptr> cb_views_vec;

  CheckerboardViewsExtraction cb_extractor;
  cb_extractor.setColorSensorPose(color_sensor_->pose());
  cb_extractor.setCheckerboardVector(cb_vec_);
  cb_extractor.setInputData(und_data_vec_);
  cb_extractor.extractAll(cb_views_vec);

  ceres::Problem problem;
  Eigen::Matrix<Scalar, Eigen::Dynamic, 6, Eigen::DontAlign | Eigen::RowMajor> data(cb_views_vec.size(), 6);

  Eigen::Matrix<Scalar, 1, 3, Eigen::DontAlign | Eigen::RowMajor> normal =
      cb_views_vec[0]->colorCheckerboard()->plane().normal();

  for (Size1 i = 0; i < cb_views_vec.size(); ++i)
  {
    const CheckerboardViews & cb_views = *cb_views_vec[i];

    AngleAxis rotation(cb_views.colorCheckerboard()->pose().linear());
    data.row(i).head<3>() = rotation.angle() * rotation.axis();
    data.row(i).tail<3>() = cb_views.colorCheckerboard()->pose().translation();

    NormalError * error = new NormalError(color_sensor_->cameraModel(),
                                          cb_views.checkerboard(),
                                          cb_views.colorView()->points());

    typedef ceres::NumericDiffCostFunction<NormalError, ceres::CENTRAL, ceres::DYNAMIC, 3, 6> NormalCostFunction;

    ceres::CostFunction * cost_function = new NormalCostFunction(error,
                                                                 ceres::DO_NOT_TAKE_OWNERSHIP,
                                                                 1 + cb_views.checkerboard()->size());
    problem.AddResidualBlock(cost_function, NULL, normal.data(), data.row(i).data());
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_SCHUR;
  options.max_num_iterations = 100;
  options.minimizer_progress_to_stdout = true;
  options.num_threads = 8;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::vector<Plane> plane_vec;
  for (size_t i = 0; i < cb_views_vec.size(); ++i)
  {
    const CheckerboardViews & cb_views = *cb_views_vec[i];
    Checkerboard cb(*cb_views.checkerboard());
    AngleAxis rotation;
    rotation.angle() = data.row(i).head<3>().norm();
    rotation.axis() = data.row(i).head<3>().normalized();
    Translation3 translation(data.row(i).tail<3>());
    cb.transform(translation * rotation);
    plane_vec.push_back(cb.plane());
  }

  for (size_t i = 0; i < cb_views_vec.size(); ++i)
  {
    const CheckerboardViews & cb_views = *cb_views_vec[i];
    RGBDData::ConstPtr und_data = cb_views.data();
    RGBDData::ConstPtr part_und_data = part_data_map_.at(und_data);
    RGBDData::ConstPtr data = data_map_.at(und_data);
    Plane plane = plane_vec[i]; //cb_views.colorCheckerboard()->plane();
    plane.transform(color_sensor_->pose());

    const Cloud3 & und_points = PCLConversion<Scalar>::toPointMatrix(*cb_views.depthView()->data(), cb_views.depthView()->points());

    Scalar d_mean = 0;
    Scalar und_mean = 0;
    for (int p = 0; p < und_points.elements(); ++p)
    {
      d_mean += und_points[p].z();
      Scalar d = plane.absDistance(und_points[p]);
      und_mean += d;
    }

    d_mean /= und_points.elements();
    und_mean /= und_points.elements();

    const Cloud3 points = PCLConversion<Scalar>::toPointMatrix(*data->depthData(), *cb_views.planeInliers());

    Scalar mean = 0;
    for (Size1 p = 0; p < points.elements(); ++p)
    {
      Scalar d = plane.absDistance(points[p]);
      mean += d;
    }

    mean /= points.elements();

    const Cloud3 part_points = PCLConversion<Scalar>::toPointMatrix(*part_und_data->depthData(), *cb_views.planeInliers());

    Scalar part_mean = 0;
    for (int p = 0; p < part_points.elements(); ++p)
    {
      Scalar d = plane.absDistance(part_points[p]);
      part_mean += d;
    }

    part_mean /= part_points.elements();

    ROS_INFO_STREAM(d_mean << " " << mean << " ** " << und_mean << " ** " << part_mean);
  }

}

} /* namespace calibration */
