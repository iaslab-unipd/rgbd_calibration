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

#ifndef RGBD_CALIBRATION_PLANE_BASED_EXTRINSIC_CALIBRATION_H_
#define RGBD_CALIBRATION_PLANE_BASED_EXTRINSIC_CALIBRATION_H_

#include <calibration_common/algorithms/plane_to_plane_calibration.h>
#include <calibration_common/objects/sensor.h>
#include <calibration_common/objects/planar_object.h>

namespace calibration
{

class PlaneBasedExtrinsicCalibration
{
public:

  typedef boost::shared_ptr<PlaneBasedExtrinsicCalibration> Ptr;
  typedef boost::shared_ptr<const PlaneBasedExtrinsicCalibration> ConstPtr;

  void addData(size_t index,
               const Sensor::Ptr & sensor,
               const PlanarObject::ConstPtr & data)
  {
    assert(index < data_map_.size());
    data_map_[index][sensor] = data;
  }

  void setSize(size_t size)
  {
    data_map_.resize(size);
  }

  size_t size() const
  {
    return data_map_.size();
  }

  size_t appendData(const std::map<Sensor::Ptr, PlanarObject::ConstPtr> & data)
  {
    data_map_.push_back(data);
    return data_map_.size();
  }

  void setMainSensor(const Sensor::Ptr & world)
  {
    world_ = world;
  }

  void perform()
  {
    std::map<Sensor::Ptr, PlaneToPlaneCalibration> calib_map;
    for (size_t i = 0; i < data_map_.size(); ++i)
    {
      std::map<Sensor::Ptr, PlanarObject::ConstPtr> & data_map = data_map_[i];
      if (data_map.find(world_) == data_map.end())
        continue;
      const PlanarObject::ConstPtr & world_data = data_map_[i][world_];
      for (std::map<Sensor::Ptr, PlanarObject::ConstPtr>::const_iterator it = data_map.begin(); it != data_map.end(); ++it)
      {
        const Sensor::Ptr & sensor = it->first;
        const PlanarObject::ConstPtr & sensor_data = it->second;
        if (sensor != world_)
          calib_map[sensor].addPair(world_data->plane(), sensor_data->plane());
      }

    }

    for (std::map<Sensor::Ptr, PlaneToPlaneCalibration>::iterator it = calib_map.begin(); it != calib_map.end(); ++it)
    {
      const Sensor::Ptr & sensor = it->first;

      if (calib_map[sensor].getPairNumber() > 5)
      {
        sensor->setParent(world_);
        sensor->setPose(calib_map[sensor].estimateTransform());
      }
      else
        sensor->setParent(Sensor::ConstPtr());
    }

  }

  void optimize()
  {
    // TODO implement!!
  }

protected:

  Sensor::Ptr world_;
  std::vector<std::map<Sensor::Ptr, PlanarObject::ConstPtr> > data_map_;

};

} /* namespace calibration */
#endif /* RGBD_CALIBRATION_PLANE_BASED_EXTRINSIC_CALIBRATION_H_ */
