/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <gz/msgs/double.pb.h>

#include <string>
#include <unordered_map>
#include <utility>

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/sensors/Noise.hh>
#include <gz/sensors/SensorFactory.hh>

#include <sdf/Sensor.hh>

#include <gz/sim/components/CustomSensor.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Util.hh>

#include "CustomGpuLidarSensor.hh"
#include "LidarSystem.hh"

using namespace custom;

//////////////////////////////////////////////////
void LidarSystem::PreUpdate(const gz::sim::UpdateInfo &,
    gz::sim::EntityComponentManager &_ecm)
{
  if (inited) return;
  // if (_ecm.HasComponentType(gz::sim::components::CustomSensor::typeId))
  // {
  //     std::cout << "CustomSensor component is registered in the simulation!" << std::endl;
  // }
  // else
  // {
  //     std::cout << "CustomSensor component is NOT registered!" << std::endl;
  // }
  // std::cout << "PreUpdate function called!" << std::endl;
  _ecm.Each<gz::sim::components::CustomSensor,
               gz::sim::components::ParentEntity>(
    [&](const gz::sim::Entity &_entity,
        const gz::sim::components::CustomSensor *_custom,
        const gz::sim::components::ParentEntity *_parent)->bool
      {
        std::cout << "EachNew function called!" << std::endl;
        // Get sensor's scoped name without the world
        auto sensorScopedName = gz::sim::removeParentScope(
            gz::sim::scopedName(_entity, _ecm, "::", false), "::");
        sdf::Sensor data = _custom->Data();
        data.SetName(sensorScopedName);

        // Default to scoped name as topic
        if (data.Topic().empty())
        {
          std::string topic = scopedName(_entity, _ecm) + "/lidar/points";
          data.SetTopic(topic);
        }

        gz::sensors::SensorFactory sensorFactory;
        auto sensor = sensorFactory.CreateSensor<custom::CustomGpuLidarSensor>(data);
        if (nullptr == sensor)
        {
          std::cout<<"Failed to create lidar"<<std::endl;
          gzerr << "Failed to create lidar [" << sensorScopedName << "]"
                 << std::endl;
          return false;
        }
        else{
          std::cout<<"Created lidar"<<std::endl;
          gzerr << "Created lidar!! Yaay [" << sensorScopedName << "]"
                 << std::endl;
        }


        // Set sensor parent
        auto parentName = _ecm.Component<gz::sim::components::Name>(
            _parent->Data())->Data();
        sensor->SetParent(parentName);

        // Set topic on Gazebo
        _ecm.CreateComponent(_entity,
            gz::sim::components::SensorTopic(sensor->Topic()));

        // Keep track of this sensor
        this->entitySensorMap.insert(std::make_pair(_entity,
            std::move(sensor)));
          inited = true;
        return true;
      });
}

//////////////////////////////////////////////////
void LidarSystem::PostUpdate(const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
  // Only update and publish if not paused.
  if (!_info.paused)
  {
    for (auto &[entity, sensor] : this->entitySensorMap)
    {
      // sensor->NewPosition(gz::sim::worldPose(entity, _ecm).Pos());
      sensor->Update(_info.simTime);
    }
  }

  this->RemoveSensorEntities(_ecm);
}

//////////////////////////////////////////////////
void LidarSystem::RemoveSensorEntities(
    const gz::sim::EntityComponentManager &_ecm)
{
  _ecm.EachRemoved<gz::sim::components::CustomSensor>(
    [&](const gz::sim::Entity &_entity,
        const gz::sim::components::CustomSensor *)->bool
      {
        if (this->entitySensorMap.erase(_entity) == 0)
        {
          gzerr << "Internal error, missing lidar for entity ["
                         << _entity << "]" << std::endl;
        }
        return true;
      });
}

GZ_ADD_PLUGIN(custom::LidarSystem, 
  gz::sim::System,
  custom::LidarSystem::ISystemPreUpdate,
  custom::LidarSystem::ISystemPostUpdate
)

GZ_ADD_PLUGIN_ALIAS(LidarSystem, "custom::LidarSystem")