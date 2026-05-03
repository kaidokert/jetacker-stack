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

#include "WorldPosePublisher.hh"

#include <gz/msgs/pose.pb.h>

#include <string>
#include <vector>
#include <sstream>
#include <unordered_map>

#include <gz/common/Profiler.hh>
#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/Util.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/Conversions.hh"

using namespace gz;
using namespace sim;
using namespace systems;

/// \brief Private data class for WorldPosePublisher
class gz::sim::systems::WorldPosePublisherPrivate
{
  /// \brief Gazebo communication node
  public: transport::Node node;

  /// \brief World entity
  public: Entity worldEntity{kNullEntity};

  /// \brief World name
  public: std::string worldName;

  /// \brief Model names to publish
  public: std::vector<std::string> modelNames;

  /// \brief Map of model name to publisher
  public: std::unordered_map<std::string, transport::Node::Publisher> publishers;

  /// \brief Map of model name to entity (cached after first lookup)
  public: std::unordered_map<std::string, Entity> modelEntities;

  /// \brief Update frequency in Hz (negative = as fast as possible)
  public: double updateFrequency = 50.0;

  /// \brief Update period calculated from frequency
  public: std::chrono::steady_clock::duration updatePeriod{0};

  /// \brief Last time poses were published
  public: std::chrono::steady_clock::duration lastPubTime{0};

  /// \brief Whether the plugin has been initialized
  public: bool initialized{false};

  /// \brief Reusable pose message (avoid allocations)
  public: msgs::Pose poseMsg;

  /// \brief Helper to parse comma-separated model names
  public: std::vector<std::string> ParseModelNames(const std::string &_input);

  /// \brief Initialize publishers for each model
  public: void InitializePublishers(const EntityComponentManager &_ecm);

  /// \brief Publish pose for a single model
  public: void PublishModelPose(const std::string &_modelName,
                                  const Entity &_entity,
                                  const EntityComponentManager &_ecm,
                                  const msgs::Time &_stampMsg);
};

//////////////////////////////////////////////////
std::vector<std::string> WorldPosePublisherPrivate::ParseModelNames(
    const std::string &_input)
{
  std::vector<std::string> result;
  std::stringstream ss(_input);
  std::string item;

  while (std::getline(ss, item, ','))
  {
    // Trim whitespace
    size_t start = item.find_first_not_of(" \t\n\r");
    size_t end = item.find_last_not_of(" \t\n\r");

    if (start != std::string::npos && end != std::string::npos)
    {
      result.push_back(item.substr(start, end - start + 1));
    }
  }

  return result;
}

//////////////////////////////////////////////////
void WorldPosePublisherPrivate::InitializePublishers(
    const EntityComponentManager &_ecm)
{
  if (this->initialized)
    return;

  // Get world name
  auto worldNameComp = _ecm.Component<components::Name>(this->worldEntity);
  if (!worldNameComp)
  {
    gzerr << "WorldPosePublisher: Failed to get world name" << std::endl;
    return;
  }
  this->worldName = worldNameComp->Data();

  gzdbg << "WorldPosePublisher: Initializing for world '" << this->worldName
        << "' with " << this->modelNames.size() << " models" << std::endl;

  // Create publisher for each model
  for (const auto &modelName : this->modelNames)
  {
    std::string topic = "/world/" + this->worldName + "/model/" + modelName + "/pose";
    this->publishers[modelName] = this->node.Advertise<msgs::Pose>(topic);
    gzdbg << "WorldPosePublisher: Created publisher for " << topic << std::endl;
  }

  this->initialized = true;
}

//////////////////////////////////////////////////
void WorldPosePublisherPrivate::PublishModelPose(
    const std::string &_modelName,
    const Entity &_entity,
    const EntityComponentManager &_ecm,
    const msgs::Time &_stampMsg)
{
  // Get model world pose from Pose component
  // Note: In Gazebo ECS, components::Pose contains the world pose for models
  auto poseComp = _ecm.Component<components::Pose>(_entity);
  if (!poseComp)
  {
    // Model exists but no pose component (shouldn't happen)
    return;
  }

  const math::Pose3d &pose = poseComp->Data();

  // Fill message
  this->poseMsg.Clear();
  auto header = this->poseMsg.mutable_header();
  header->mutable_stamp()->CopyFrom(_stampMsg);

  // Frame information
  auto frameData = header->add_data();
  frameData->set_key("frame_id");
  frameData->add_value(this->worldName);

  auto childFrameData = header->add_data();
  childFrameData->set_key("child_frame_id");
  childFrameData->add_value(_modelName);

  // Set pose
  this->poseMsg.set_name(_modelName);
  msgs::Set(&this->poseMsg, pose);

  // Publish
  auto it = this->publishers.find(_modelName);
  if (it != this->publishers.end())
  {
    it->second.Publish(this->poseMsg);
  }
}

//////////////////////////////////////////////////
WorldPosePublisher::WorldPosePublisher()
  : dataPtr(std::make_unique<WorldPosePublisherPrivate>())
{
}

//////////////////////////////////////////////////
void WorldPosePublisher::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  // Verify this is attached to a world entity
  auto world = _ecm.Component<components::World>(_entity);
  if (!world)
  {
    gzerr << "WorldPosePublisher plugin should be attached to a world entity. "
          << "Failed to initialize." << std::endl;
    return;
  }

  this->dataPtr->worldEntity = _entity;

  // Parse model names
  std::string modelNamesStr = _sdf->Get<std::string>("model_names", "").first;
  if (modelNamesStr.empty())
  {
    gzwarn << "WorldPosePublisher: No model_names specified. "
           << "Plugin will not publish any poses." << std::endl;
    return;
  }

  this->dataPtr->modelNames = this->dataPtr->ParseModelNames(modelNamesStr);

  if (this->dataPtr->modelNames.empty())
  {
    gzwarn << "WorldPosePublisher: Failed to parse model_names. "
           << "Plugin will not publish any poses." << std::endl;
    return;
  }

  // Parse update frequency
  this->dataPtr->updateFrequency =
      _sdf->Get<double>("update_frequency", this->dataPtr->updateFrequency).first;

  if (this->dataPtr->updateFrequency > 0)
  {
    std::chrono::duration<double> period{1.0 / this->dataPtr->updateFrequency};
    this->dataPtr->updatePeriod =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
  }

  gzmsg << "WorldPosePublisher configured:" << std::endl;
  gzmsg << "  Models: ";
  for (size_t i = 0; i < this->dataPtr->modelNames.size(); ++i)
  {
    gzmsg << this->dataPtr->modelNames[i];
    if (i < this->dataPtr->modelNames.size() - 1)
      gzmsg << ", ";
  }
  gzmsg << std::endl;
  gzmsg << "  Update frequency: " << this->dataPtr->updateFrequency << " Hz"
        << std::endl;
}

//////////////////////////////////////////////////
void WorldPosePublisher::PostUpdate(const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  GZ_PROFILE("WorldPosePublisher::PostUpdate");

  // Skip if paused
  if (_info.paused)
    return;

  // Check update frequency
  if (this->dataPtr->updatePeriod > std::chrono::steady_clock::duration::zero())
  {
    auto diff = _info.simTime - this->dataPtr->lastPubTime;
    if (diff < this->dataPtr->updatePeriod)
      return;
  }

  // Initialize on first update
  if (!this->dataPtr->initialized)
  {
    this->dataPtr->InitializePublishers(_ecm);
  }

  // Get timestamp
  auto stampMsg = convert<msgs::Time>(_info.simTime);

  // Publish each model's pose
  for (const auto &modelName : this->dataPtr->modelNames)
  {
    Entity modelEntity = kNullEntity;

    // Check cache first
    auto cacheIt = this->dataPtr->modelEntities.find(modelName);
    if (cacheIt != this->dataPtr->modelEntities.end())
    {
      modelEntity = cacheIt->second;

      // Verify entity still exists (may have been destroyed during reset)
      if (!_ecm.HasEntity(modelEntity))
      {
        // Entity was destroyed, need to re-query
        this->dataPtr->modelEntities.erase(cacheIt);
        modelEntity = kNullEntity;
      }
    }

    // Query if not in cache or cache was invalidated
    if (modelEntity == kNullEntity)
    {
      modelEntity = _ecm.EntityByComponents(
          components::Model(),
          components::Name(modelName));

      if (modelEntity == kNullEntity)
      {
        // Model not found - this is expected during initialization or after reset
        // Don't spam warnings
        continue;
      }

      // Cache the entity
      this->dataPtr->modelEntities[modelName] = modelEntity;
      gzdbg << "WorldPosePublisher: Found model '" << modelName
            << "' with entity " << modelEntity << std::endl;
    }

    // Publish pose
    this->dataPtr->PublishModelPose(modelName, modelEntity, _ecm, stampMsg);
  }

  this->dataPtr->lastPubTime = _info.simTime;
}

GZ_ADD_PLUGIN(WorldPosePublisher,
              System,
              WorldPosePublisher::ISystemConfigure,
              WorldPosePublisher::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(WorldPosePublisher,
                    "gz::sim::systems::WorldPosePublisher")
