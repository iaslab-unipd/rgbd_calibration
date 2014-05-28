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

#ifndef RGBD_CALIBRATION_CHECKERBOARD_VIEWS_EXTRACTOR_H_
#define RGBD_CALIBRATION_CHECKERBOARD_VIEWS_EXTRACTOR_H_

#include <vector>
#include <rgbd_calibration/checkerboard_views.h>

namespace calibration
{

class CheckerboardViewsExtractor
{
public:

  typedef ColorView<PinholeSensor, Checkerboard> CheckerboardView;
  typedef DepthViewEigen<PlanarObject> PlaneView;

  CheckerboardViewsExtractor()
    : force_(false),
      only_images_(false),
      color_sensor_pose_(Pose::Identity()),
      cb_constraint_(boost::make_shared<NoConstraint<Checkerboard> >()),
      plane_constraint_(boost::make_shared<NoConstraint<PlanarObject> >())
  {
  }

  void setCheckerboardVector(const std::vector<Checkerboard::ConstPtr> & cb_vec)
  {
    cb_vec_ = cb_vec;
  }

  void addCheckerboard(const Checkerboard::ConstPtr & checkerboard)
  {
    cb_vec_.push_back(checkerboard);
  }

  void setCheckerboardConstraint(const Constraint<Checkerboard>::ConstPtr & cb_constraint)
  {
    cb_constraint_ = cb_constraint;
  }

  void setPlanarObjectConstraint(const Constraint<PlanarObject>::ConstPtr & plane_constraint)
  {
    plane_constraint_ = plane_constraint;
  }

  void setInputData(const RGBDData::ConstPtr & data)
  {
    data_ = data;
  }

  void setInputData(const std::vector<RGBDData::ConstPtr> & data_vec)
  {
    data_vec_ = data_vec;
  }

  void setForceAll(bool force)
  {
    force_ = force;
  }

  void setOnlyImages(bool only_images)
  {
    only_images_ = only_images;
  }

  void setColorSensorPose(const Pose & color_sensor_pose)
  {
    color_sensor_pose_ = color_sensor_pose;
  }

  size_t extract(std::vector<CheckerboardViews::Ptr> & cb_views_vec,
                 bool interactive = false) const;

  size_t extractAll(std::vector<CheckerboardViews::Ptr> & cb_views_vec,
                    bool interactive = false) const;

private:

  size_t extract(const RGBDData::ConstPtr & data,
                 std::vector<CheckerboardViews::Ptr> & cb_views_vec,
                 bool interactive,
                 bool force) const;

  std::vector<Checkerboard::ConstPtr> cb_vec_;

  RGBDData::ConstPtr data_;
  std::vector<RGBDData::ConstPtr> data_vec_;

  bool force_;
  bool only_images_;
  Pose color_sensor_pose_;

  Constraint<Checkerboard>::ConstPtr cb_constraint_;
  Constraint<PlanarObject>::ConstPtr plane_constraint_;

};

} /* namespace calibration */
#endif /* RGBD_CALIBRATION_CHECKERBOARD_VIEWS_EXTRACTOR_H_ */
