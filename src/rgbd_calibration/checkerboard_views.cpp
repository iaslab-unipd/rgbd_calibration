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

#include <calibration_common/base/pcl_conversion.h>
#include <calibration_common/ceres/plane_fit.h>

#include <rgbd_calibration/checkerboard_views.h>

namespace calibration
{

void CheckerboardViews::setColorView(const PinholeView<Checkerboard>::ConstPtr & color_view)
{
  color_view_ = boost::make_shared<PinholeView<Checkerboard> >(*color_view);
  //  color_checkerboard_ = boost::make_shared<Checkerboard>(*color_view_);

  std::stringstream ss;
  ss << checkerboard_->frameId() << "_" << id_;

  color_checkerboard_ = boost::make_shared<Checkerboard>(*checkerboard_);
  color_checkerboard_->setParent(data_->colorSensor());
  color_checkerboard_->setFrameId(ss.str());
  color_checkerboard_->transform(color_view_->sensor()->estimatePose(color_view_->points(), checkerboard_->corners()));
}

void CheckerboardViews::setImageCorners(const Cloud2 & image_corners)
{
  PinholeView<Checkerboard>::Ptr color_view = boost::make_shared<PinholeView<Checkerboard> >();
  color_view->setId(id_);
  color_view->setPoints(image_corners);
  color_view->setSensor(data_->colorSensor());
  color_view->setObject(checkerboard_);

  setColorView(color_view);
}

void CheckerboardViews::setPlaneInliers(const pcl::IndicesConstPtr & plane_inliers,
                                        Scalar inliers_std_dev)
{
  plane_inliers_ = plane_inliers;

  depth_view_ = boost::make_shared<DepthViewPCL<PlanarObject> >();
  depth_view_->setId(id_);
  depth_view_->setData(data_->depthData());
  depth_view_->setPoints(PCLConversion<Scalar>::toPointMatrix(*data_->depthData(), *plane_inliers));
  depth_view_->setSensor(data_->depthSensor());
  depth_view_->setObject(checkerboard_);
  depth_view_->setPointsStdDev(inliers_std_dev);

  depth_plane_ = boost::make_shared<PlanarObject>();
  depth_plane_->setParent(data_->depthSensor());

  std::stringstream ss;
  ss << checkerboard_->frameId() << "_plane_" << id_;
  depth_plane_->setFrameId(ss.str());

  depth_plane_->setPlane(PlaneFit<Scalar>::robustFit(depth_view_->points(), inliers_std_dev));
  //depth_plane_->setPlane(PlaneFit<Scalar>::fit(depth_view_->points()));
}

void CheckerboardViews::setPlaneInliers(const PlaneInfo & plane_info)
{
  plane_inliers_ = plane_info.indices_;

  depth_view_ = boost::make_shared<DepthViewPCL<PlanarObject> >();
  depth_view_->setId(id_);
  depth_view_->setData(data_->depthData());
  depth_view_->setPoints(PCLConversion<Scalar>::toPointMatrix(*data_->depthData(), *plane_info.indices_));
  depth_view_->setSensor(data_->depthSensor());
  depth_view_->setObject(checkerboard_);
  depth_view_->setPointsStdDev(plane_info.std_dev_);

  depth_plane_ = boost::make_shared<PlanarObject>();
  depth_plane_->setParent(data_->depthSensor());

  std::stringstream ss;
  ss << checkerboard_->frameId() << "_plane_" << id_;
  depth_plane_->setFrameId(ss.str());

  depth_plane_->setPlane(plane_info.plane_);
}

void CheckerboardViews::draw(cv::Mat & image) const
{
  cv::Size pattern_size(color_checkerboard_->rows(), color_checkerboard_->cols());
  std::vector<cv::Point2f> corners;
  for (int i = 0; i < color_view_->points().size().prod(); ++i)
    corners.push_back(cv::Point2f(color_view_->points()[i][0], color_view_->points()[i][1]));

  cv::drawChessboardCorners(image, pattern_size, cv::Mat(corners), true);
}

} /* namespace calibration */
