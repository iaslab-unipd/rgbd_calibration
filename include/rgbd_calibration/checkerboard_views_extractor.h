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

#ifndef RGBD_CALIBRATION_CHECKERBOARD_VIEWS_EXTRACTOR_H_
#define RGBD_CALIBRATION_CHECKERBOARD_VIEWS_EXTRACTOR_H_

#include <vector>
#include <rgbd_calibration/checkerboard_views.h>

namespace calibration
{

class CheckerboardViewsExtraction
{
public:

  typedef ColorView<PinholeSensor, Checkerboard> CheckerboardView;
  typedef DepthViewEigen<PlanarObject> PlaneView;

  CheckerboardViewsExtraction()
    : force_(false),
      only_images_(false),
      color_sensor_pose_(Pose::Identity()),
      cb_constraint_(boost::make_shared<NoConstraint<Checkerboard> >()),
      plane_constraint_(boost::make_shared<NoConstraint<PlanarObject> >())
  {
  }

  inline void setCheckerboardVector(const std::vector<Checkerboard::ConstPtr> & cb_vec)
  {
    cb_vec_ = cb_vec;
  }

  inline void addCheckerboard(const Checkerboard::ConstPtr & checkerboard)
  {
    cb_vec_.push_back(checkerboard);
  }

  inline void setCheckerboardConstraint(const Constraint<Checkerboard>::ConstPtr & cb_constraint)
  {
    cb_constraint_ = cb_constraint;
  }

  inline void setPlanarObjectConstraint(const Constraint<PlanarObject>::ConstPtr & plane_constraint)
  {
    plane_constraint_ = plane_constraint;
  }

  inline void setInputData(const RGBDData::ConstPtr & data)
  {
    data_ = data;
  }

  inline void setInputData(const std::vector<RGBDData::ConstPtr> & data_vec)
  {
    data_vec_ = data_vec;
  }

  inline void setForceAll(bool force)
  {
    force_ = force;
  }

  inline void setOnlyImages(bool only_images)
  {
    only_images_ = only_images;
  }

  inline void setColorSensorPose(const Pose & color_sensor_pose)
  {
    color_sensor_pose_ = color_sensor_pose;
  }

  Size1 extract(std::vector<CheckerboardViews::Ptr> & cb_views_vec,
                bool interactive = false) const;

  Size1 extractAll(std::vector<CheckerboardViews::Ptr> & cb_views_vec,
                   bool interactive = false) const;

private:

  Size1 extract(const RGBDData::ConstPtr & data,
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
