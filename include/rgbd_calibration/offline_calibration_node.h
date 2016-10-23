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

#ifndef RGBD_CALIBRATION_OFFLINE_CALIBRATION_NODE_H_
#define RGBD_CALIBRATION_OFFLINE_CALIBRATION_NODE_H_

#include <rgbd_calibration/calibration_node.h>

namespace calibration
{

enum DepthType
{
  KINECT1_DEPTH,
  SWISS_RANGER_DEPTH
};

class OfflineCalibrationNode : public CalibrationNode
{
public:

  OfflineCalibrationNode (ros::NodeHandle & node_handle);

  virtual
  ~OfflineCalibrationNode()
  {
    // Do nothing
  }

  virtual void
  spin ();

protected:

  int instances_;
  int starting_index_;

  std::string path_;
  std::string image_extension_;
  std::string image_filename_;
  std::string cloud_filename_;

  DepthType depth_type_;

};

} /* namespace calibration */
#endif /* RGBD_CALIBRATION_OFFLINE_CALIBRATION_NODE_H_ */
