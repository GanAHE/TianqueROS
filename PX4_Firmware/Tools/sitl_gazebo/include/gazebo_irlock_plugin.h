#ifndef _GAZEBO_IRLOCK_PLUGIN_HH_
#define _GAZEBO_IRLOCK_PLUGIN_HH_

#include <string>

#include "gazebo/common/Plugin.hh"
#include "gazebo/sensors/LogicalCameraSensor.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/util/system.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"

#include "IRLock.pb.h"

#include <iostream>

using namespace std;

namespace gazebo
{
  class GAZEBO_VISIBLE IRLockPlugin : public SensorPlugin
  {
    public:
      IRLockPlugin();
      virtual ~IRLockPlugin();
      virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);
      virtual void OnUpdated();

    protected:
      sensors::LogicalCameraSensorPtr camera;
      physics::WorldPtr world;

    private:
      event::ConnectionPtr updateConnection;
      transport::PublisherPtr irlock_pub_;
      transport::NodePtr node_handle_;
      sensor_msgs::msgs::IRLock irlock_message;
      std::string namespace_;

  };
}
#endif
