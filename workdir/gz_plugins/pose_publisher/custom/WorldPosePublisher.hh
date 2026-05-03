/*
 * Copyright (C) 2026 Custom Implementation
 * Based on PosePublisher from Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#ifndef GZ_SIM_SYSTEMS_WORLDPOSEPUBLISHER_HH_
#define GZ_SIM_SYSTEMS_WORLDPOSEPUBLISHER_HH_

#include <memory>
#include <gz/sim/config.hh>
#include <gz/sim/System.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  // Forward declaration
  class WorldPosePublisherPrivate;

  /// \brief World-level pose publisher system that survives world resets.
  ///
  /// Unlike the standard PosePublisher which attaches to models and is
  /// destroyed during world reset, this plugin attaches to the world entity
  /// and continues publishing after resets.
  ///
  /// ## System Parameters
  ///
  /// - `<model_names>`: Comma-separated list of model names to publish
  /// - `<update_frequency>`: Frequency of pose publications in Hz (default: 50)
  ///   A negative frequency publishes as fast as possible
  ///
  /// ## Topics Published
  ///
  /// For each model: `/world/<world_name>/model/<model_name>/pose` (gz.msgs.Pose)
  ///
  /// ## Example Usage
  ///
  /// ```xml
  /// <world name="my_world">
  ///   <plugin
  ///     filename="libWorldPosePublisher"
  ///     name="gz::sim::systems::WorldPosePublisher">
  ///     <model_names>slam_bot,jetacker</model_names>
  ///     <update_frequency>50</update_frequency>
  ///   </plugin>
  /// </world>
  /// ```
  class WorldPosePublisher
    : public System,
      public ISystemConfigure,
      public ISystemPostUpdate
  {
    /// \brief Constructor
    public: WorldPosePublisher();

    /// \brief Destructor
    public: ~WorldPosePublisher() override = default;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PostUpdate(
                 const UpdateInfo &_info,
                 const EntityComponentManager &_ecm) override;

    /// \brief Private data pointer
    private: std::unique_ptr<WorldPosePublisherPrivate> dataPtr;
  };
}
}
}
}

#endif
