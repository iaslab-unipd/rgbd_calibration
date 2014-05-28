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

#ifndef RGBD_CALIBRATION_SIMULATION_NODE_H_
#define RGBD_CALIBRATION_SIMULATION_NODE_H_

#include <rgbd_calibration/calibration_node.h>

namespace calibration
{

class SimulationNode : public CalibrationNode
{
public:

  SimulationNode(ros::NodeHandle & node_handle);

  virtual void spin();

protected:

  double depth_error_;
  double image_error_;

  double min_distance_;
  double max_distance_;
  double step_;

};

} /* namespace calibration */
#endif /* RGBD_CALIBRATION_SIMULATION_NODE_H_ */
