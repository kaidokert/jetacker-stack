/*
 * Copyright (C) 2026 Custom Implementation
 * Based on WorldPosePublisher (which was based on Open Source Robotics Foundation's PosePublisher)
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
#ifndef GZ_SIM_SYSTEMS_WORLDSTATEPUBLISHER_HH_
#define GZ_SIM_SYSTEMS_WORLDSTATEPUBLISHER_HH_

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
  class WorldStatePublisherPrivate;

  /// \brief World-level state publisher system that survives world resets.
  ///
  /// This plugin publishes both model poses and joint states for specified models.
  /// Unlike model-level plugins which are destroyed during world reset, this plugin
  /// attaches to the world entity and continues publishing after resets.
  ///
  /// ## System Parameters
  ///
  /// - `<update_frequency>`: Frequency of publications in Hz (default: 50)
  ///
  /// - `<model>`: Model configuration (can specify multiple)
  ///   - `<name>`: Model name (required)
  ///   - `<publish_pose>`: Publish pose (default: true)
  ///   - `<publish_joints>`: Publish joint states (default: false)
  ///   - `<joint>`: Joint name to publish (can specify multiple, empty = all joints)
  ///
  /// ## Topics Published
  ///
  /// For each model:
  /// - Pose: `/world/<world_name>/model/<model_name>/pose` (gz.msgs.Pose)
  /// - Joints: `/world/<world_name>/model/<model_name>/joint_states` (gz.msgs.Model)
  ///
  /// ## Example Usage
  ///
  /// ```xml
  /// <world name="my_world">
  ///   <plugin
  ///     filename="libWorldStatePublisher"
  ///     name="gz::sim::systems::WorldStatePublisher">
  ///     <update_frequency>50</update_frequency>
  ///
  ///     <model>
  ///       <name>jetacker</name>
  ///       <publish_pose>true</publish_pose>
  ///       <publish_joints>true</publish_joints>
  ///       <joint>front_left_wheel_steering_joint</joint>
  ///       <joint>front_right_wheel_steering_joint</joint>
  ///       <joint>rear_left_wheel_joint</joint>
  ///       <joint>rear_right_wheel_joint</joint>
  ///     </model>
  ///
  ///     <model>
  ///       <name>obstacle_1</name>
  ///       <publish_pose>true</publish_pose>
  ///       <publish_joints>false</publish_joints>
  ///     </model>
  ///   </plugin>
  /// </world>
  /// ```
  class WorldStatePublisher
    : public System,
      public ISystemConfigure,
      public ISystemPreUpdate
  {
    /// \brief Constructor
    public: WorldStatePublisher();

    /// \brief Destructor
    public: ~WorldStatePublisher() override = default;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PreUpdate(
                 const UpdateInfo &_info,
                 EntityComponentManager &_ecm) override;

    /// \brief Private data pointer
    private: std::unique_ptr<WorldStatePublisherPrivate> dataPtr;
  };
}
}
}
}

#endif
