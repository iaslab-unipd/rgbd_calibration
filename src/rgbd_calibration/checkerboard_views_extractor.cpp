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

#include <rgbd_calibration/checkerboard_views_extractor.h>

#include <calibration_common/algorithms/plane_extractor.h>
#include <calibration_common/algorithms/interactive_checkerboard_finder.h>
#include <calibration_common/algorithms/automatic_checkerboard_finder.h>

namespace calibration
{

size_t CheckerboardViewsExtractor::extract(const RGBDData::ConstPtr & data,
                                           std::vector<CheckerboardViews::Ptr> & cb_views_vec,
                                           bool interactive,
                                           bool force) const
{
  size_t added = 0;

  cv::Mat image = data->colorData().clone();
  AutomaticCheckerboardFinder finder;
  finder.setImage(image);

  PointPlaneExtractor<PCLPoint3>::Ptr plane_extractor;

  if (not interactive)
    plane_extractor = boost::make_shared<PointPlaneExtractor<PCLPoint3> >();
  else
    plane_extractor = boost::make_shared<PointPlaneExtractorGUI<PCLPoint3> >();

  plane_extractor->setInputCloud(data->depthData());

  //int base_id = data->id() * cb_vec_.size();

  for (size_t c = 0; c < cb_vec_.size(); ++c)
  {
    const Checkerboard::ConstPtr & cb = cb_vec_[c];

    plane_extractor->setRadius(std::min(cb->width(), cb->height()) / 1.8);

    std::stringstream ss;
    ss << "rgbd_cb_" << data->id() << "_" << c;
    CheckerboardViews::Ptr cb_views(boost::make_shared<CheckerboardViews>(ss.str()));
    cb_views->setData(data);
    cb_views->setCheckerboard(cb);

    // 1. Extract corners

    Cloud2 image_corners(cb->rows(), cb->cols());
    if (not finder.find(*cb, image_corners))
    {
      if (force)
      {
        InteractiveCheckerboardFinder finder2(image);
        if (not finder2.find(*cb, image_corners))
          continue;
      }
      else
        continue;
    }

    cb_views->setImageCorners(image_corners);

    if (not cb_constraint_->isValid(*cb_views->colorCheckerboard()))
      continue;

    // 2. Extract plane

    if (not only_images_)
    {
      PlaneInfo plane_info;

      if (interactive)
      {
        cb_views->draw(image);
      }
      else
      {
        Point3 center = color_sensor_pose_ * cb_views->colorCheckerboard()->center();
        PCLPoint3 p;
        p.x = center[0];
        p.y = center[1];
        p.z = center[2];
        plane_extractor->setPoint(p);
      }

      if (not plane_extractor->extract(plane_info))
        continue;

      cb_views->setPlaneInliers(plane_info.indices_, plane_info.std_dev_);

      if (not plane_constraint_->isValid(*cb_views->depthPlane()))
        continue;

    }

#pragma omp critical
    cb_views_vec.push_back(cb_views);

    ++added;
  }

  return added;

}

size_t CheckerboardViewsExtractor::extract(std::vector<CheckerboardViews::Ptr> & cb_views_vec,
                                           bool interactive) const
{
  return extract(data_, cb_views_vec, interactive, true);
}

size_t CheckerboardViewsExtractor::extractAll(std::vector<CheckerboardViews::Ptr> & cb_views_vec,
                                              bool interactive) const
{
  size_t added = 0;

#pragma omp parallel for
  for (size_t i = 0; i < data_vec_.size(); ++i)
  {
    size_t n = extract(data_vec_[i], cb_views_vec, interactive, force_);
#pragma omp atomic
    added += n;
  }

  return added;
}

} /* namespace calibration */
