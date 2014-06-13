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

#ifndef RGBD_CALIBRATION_CHECKERBOARD_VIEWS_H_
#define RGBD_CALIBRATION_CHECKERBOARD_VIEWS_H_

#include <calibration_common/pinhole/view.h>
#include <calibration_common/depth/view.h>
#include <calibration_common/objects/checkerboard.h>
#include <calibration_common/algorithms/plane_extractor.h>

#include <calibration_common/rgbd/data.h>

#include <pcl/pcl_base.h>

namespace calibration
{

class CheckerboardViews
{
public:

  typedef boost::shared_ptr<CheckerboardViews> Ptr;
  typedef boost::shared_ptr<const CheckerboardViews> ConstPtr;

  CheckerboardViews(const std::string & id)
    : id_(id)
  {
    // Do nothing
  }

  inline void setId(const std::string & id)
  {
    id_ = id;
  }

  inline const std::string & id() const
  {
    return id_;
  }

  inline void setData(const RGBDData::ConstPtr & data)
  {
    data_ = data;
  }

  inline const RGBDData::ConstPtr & data() const
  {
    return data_;
  }

  inline void setCheckerboard(const Checkerboard::ConstPtr & checkerboard)
  {
    checkerboard_ = checkerboard;
  }

  inline const Checkerboard::ConstPtr & checkerboard() const
  {
    return checkerboard_;
  }

  inline const Checkerboard::Ptr & colorCheckerboard() const
  {
    return color_checkerboard_;
  }

  inline const PlanarObject::Ptr & depthPlane() const
  {
    return depth_plane_;
  }

  inline PinholeView<Checkerboard>::ConstPtr colorView() const
  {
    return color_view_;
  }

  inline DepthViewPCL<PlanarObject>::ConstPtr depthView() const
  {
    return depth_view_;
  }

  inline const pcl::IndicesConstPtr & planeInliers() const
  {
    return plane_inliers_;
  }

  void setColorView(const PinholeView<Checkerboard>::ConstPtr & color_view);

  void setImageCorners(const Cloud2 & image_corners);

  void setPlaneInliers(const pcl::IndicesConstPtr & plane_inliers,
                       Scalar inliers_std_dev);

  void setPlaneInliers(const PlaneInfo & plane_info);

  void draw(cv::Mat & image) const;

protected:

  std::string id_;
  RGBDData::ConstPtr data_;
  Checkerboard::ConstPtr checkerboard_;

  PinholeView<Checkerboard>::Ptr color_view_;
  DepthViewPCL<PlanarObject>::Ptr depth_view_;

  Checkerboard::Ptr color_checkerboard_;
  PlanarObject::Ptr depth_plane_;

  pcl::IndicesConstPtr plane_inliers_;

};

} /* namespace calibration */
#endif /* RGBD_CALIBRATION_CHECKERBOARD_VIEWS_H_ */
