/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
/*
 * Desc: Lidar Plugin
 * Author: Nate Koenig mod by John Hsu
 */

#ifndef _GAZEBO_LIDAR_PLUGIN_HH_
#define _GAZEBO_LIDAR_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/util/system.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"

#include "gazebo/msgs/msgs.hh"
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/RaySensor.hh>

#include <common.h>

#include <Range.pb.h>

namespace gazebo
{
  static constexpr double kSensorMinDistance = 0.06;    // values smaller than that cause issues
  static constexpr double kSensorMaxDistance = 35.0;    // values bigger than that cause issues
  static constexpr double kDefaultMinDistance = 0.2;
  static constexpr double kDefaultMaxDistance = 15.0;
  static constexpr double kDefaultFOV = 0.0523598776;   // standard 3 degrees

  /// \brief A Ray Sensor Plugin
  class GAZEBO_VISIBLE LidarPlugin : public SensorPlugin
  {
    /// \brief Constructor
    public: LidarPlugin();

    /// \brief Destructor
    public: virtual ~LidarPlugin();

    /// \brief Update callback
    public: virtual void OnNewLaserScans();

    /// \brief Load the plugin
    /// \param take in SDF root element
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Pointer to parent
    protected: physics::WorldPtr world_;

    /// \brief The parent sensor
    private:
      sensors::RaySensorPtr parentSensor_;
      std::string lidar_topic_;
      transport::NodePtr node_handle_;
      transport::PublisherPtr lidar_pub_;
      std::string namespace_;
      double min_distance_;
      double max_distance_;

      gazebo::msgs::Quaternion orientation_;

    /// \brief The connection tied to LidarPlugin::OnNewLaserScans()
    private:
      event::ConnectionPtr newLaserScansConnection_;
      sensor_msgs::msgs::Range lidar_message_;
  };
}
#endif
