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

#ifndef RGBD_CALIBRATION_CHECKERBOARD_VIEWS_H_
#define RGBD_CALIBRATION_CHECKERBOARD_VIEWS_H_

#include <calibration_common/pinhole/view.h>
#include <calibration_common/depth/view.h>
#include <calibration_common/objects/checkerboard.h>
#include <calibration_common/algorithms/plane_extraction.h>

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
