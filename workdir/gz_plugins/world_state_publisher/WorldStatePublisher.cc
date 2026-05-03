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

#include "WorldStatePublisher.hh"

#include <gz/msgs/pose.pb.h>
#include <gz/msgs/model.pb.h>

#include <string>
#include <vector>
#include <set>
#include <unordered_map>
#include <unordered_set>

#include <gz/common/Profiler.hh>
#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/Util.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/JointVelocity.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/Conversions.hh"

using namespace gz;
using namespace sim;
using namespace systems;

/// \brief Joint information for caching
struct JointInfo {
  std::string name;
  Entity entity = kNullEntity;
};

/// \brief Per-model configuration
struct ModelConfig {
  std::string name;
  bool publish_pose = true;
  bool publish_joints = false;
  std::vector<std::string> joint_names;  // Whitelist (empty = all)
  mutable std::vector<JointInfo> cached_joints;  // Performance cache
};

/// \brief Plugin-level configuration
struct PluginConfig {
  double update_frequency = 50.0;
  std::vector<ModelConfig> models;
};

/// \brief Private data class for WorldStatePublisher
class gz::sim::systems::WorldStatePublisherPrivate
{
  /// \brief Gazebo communication node
  public: transport::Node node;

  /// \brief World entity
  public: Entity worldEntity{kNullEntity};

  /// \brief World name
  public: std::string worldName;

  /// \brief Plugin configuration
  public: PluginConfig config;

  /// \brief Map of model name to cached entity
  public: std::unordered_map<std::string, Entity> modelCache;

  /// \brief Map of model name to pose publisher
  public: std::unordered_map<std::string, transport::Node::Publisher> posePubs;

  /// \brief Map of model name to joint state publisher
  public: std::unordered_map<std::string, transport::Node::Publisher> jointPubs;

  /// \brief Last time data was published
  public: std::chrono::steady_clock::duration lastPubTime{0};

  /// \brief Whether the plugin has been initialized
  public: bool initialized{false};

  /// \brief Get model entity with caching and reset handling
  public: Entity GetModelEntity(const std::string &_name,
                                 const EntityComponentManager &_ecm);

  /// \brief Initialize publishers for all configured models
  public: void Initialize(const EntityComponentManager &_ecm);

  /// \brief Publish pose for a model
  public: void PublishPose(const ModelConfig &_config,
                           Entity _modelEntity,
                           const EntityComponentManager &_ecm,
                           const msgs::Time &_stampMsg);

  /// \brief Publish joint states for a model
  public: void PublishJointStates(ModelConfig &_config,
                                  Entity _modelEntity,
                                  EntityComponentManager &_ecm,
                                  const msgs::Time &_stampMsg);

  /// \brief Resolve joint entities and cache them
  public: void ResolveJointEntities(ModelConfig &_config,
                                    Entity _modelEntity,
                                    const EntityComponentManager &_ecm);
};

//////////////////////////////////////////////////
Entity WorldStatePublisherPrivate::GetModelEntity(
    const std::string &_name,
    const EntityComponentManager &_ecm)
{
  // Check cache
  auto it = this->modelCache.find(_name);
  if (it != this->modelCache.end()) {
    Entity cached = it->second;

    // Verify entity still valid (handle reset)
    if (_ecm.HasEntity(cached)) {
      return cached;
    }

    // Entity was destroyed (world reset), invalidate cache
    this->modelCache.erase(it);
  }

  // Query ECM by name
  Entity modelEntity = _ecm.EntityByComponents(
      components::Model(),
      components::Name(_name));

  if (modelEntity != kNullEntity) {
    // Cache for next iteration
    this->modelCache[_name] = modelEntity;
  } else {
    gzwarn << "WorldStatePublisher: Model '" << _name << "' not found in world\n";
  }

  return modelEntity;
}

//////////////////////////////////////////////////
void WorldStatePublisherPrivate::Initialize(const EntityComponentManager &_ecm)
{
  // Get world name
  auto worldNameComp = _ecm.Component<components::Name>(this->worldEntity);
  if (!worldNameComp) {
    gzerr << "World entity has no Name component\n";
    return;
  }
  this->worldName = worldNameComp->Data();

  // Create publishers for each configured model
  for (const auto &modelConfig : this->config.models) {
    if (modelConfig.publish_pose) {
      std::string poseTopic = "/world/" + this->worldName +
                              "/model/" + modelConfig.name + "/pose";
      this->posePubs[modelConfig.name] =
          this->node.Advertise<msgs::Pose>(poseTopic);
      gzmsg << "WorldStatePublisher: Advertising pose on " << poseTopic << "\n";
    }

    if (modelConfig.publish_joints) {
      std::string jointTopic = "/world/" + this->worldName +
                               "/model/" + modelConfig.name + "/joint_states";
      this->jointPubs[modelConfig.name] =
          this->node.Advertise<msgs::Model>(jointTopic);
      gzmsg << "WorldStatePublisher: Advertising joints on " << jointTopic << "\n";
    }
  }

  this->initialized = true;
}

//////////////////////////////////////////////////
void WorldStatePublisherPrivate::PublishPose(
    const ModelConfig &_config,
    Entity _modelEntity,
    const EntityComponentManager &_ecm,
    const msgs::Time &_stampMsg)
{
  // Verify entity still exists (may have been destroyed mid-reset)
  if (!_ecm.HasEntity(_modelEntity)) {
    return;
  }

  // Get model pose
  auto poseComp = _ecm.Component<components::Pose>(_modelEntity);
  if (!poseComp) {
    return;
  }

  // Build message
  msgs::Pose msg;
  msg.Clear();
  msgs::Set(&msg, poseComp->Data());
  msg.mutable_header()->mutable_stamp()->CopyFrom(_stampMsg);

  // Publish
  auto it = this->posePubs.find(_config.name);
  if (it != this->posePubs.end()) {
    it->second.Publish(msg);
  }
}

//////////////////////////////////////////////////
void WorldStatePublisherPrivate::PublishJointStates(
    ModelConfig &_config,
    Entity _modelEntity,
    EntityComponentManager &_ecm,
    const msgs::Time &_stampMsg)
{
  // Verify model entity is still valid (reset may have destroyed it)
  if (_modelEntity == kNullEntity || !_ecm.HasEntity(_modelEntity)) {
    _config.cached_joints.clear();
    return;
  }

  // Resolve joint entities if not cached (first call or after reset)
  bool needsResolve = _config.cached_joints.empty();
  if (!needsResolve) {
    for (const auto &j : _config.cached_joints) {
      if (j.entity == kNullEntity || !_ecm.HasEntity(j.entity)) {
        needsResolve = true;
        break;
      }
    }
  }

  if (needsResolve) {
    _config.cached_joints.clear();
    this->ResolveJointEntities(_config, _modelEntity, _ecm);

    if (_config.cached_joints.empty()) {
      return;
    }
  }

  // Create Model message
  msgs::Model msg;
  msg.Clear();

  // Set header
  auto header = msg.mutable_header();
  header->mutable_stamp()->CopyFrom(_stampMsg);

  // Set model name
  msg.set_name(_config.name);

  // Iterate ONLY cached joints
  for (auto &jointInfo : _config.cached_joints) {
    // Verify entity still valid (handle reset)
    if (jointInfo.entity == kNullEntity || !_ecm.HasEntity(jointInfo.entity)) {
      jointInfo.entity = kNullEntity;
      continue;
    }

    // Get position component — do NOT create if missing during potential reset
    auto posComp = _ecm.Component<components::JointPosition>(jointInfo.entity);
    auto velComp = _ecm.Component<components::JointVelocity>(jointInfo.entity);

    if (!posComp || !velComp) {
      // Request component creation for next iteration (only if entity confirmed valid)
      if (_ecm.HasEntity(jointInfo.entity)) {
        if (!posComp)
          _ecm.CreateComponent(jointInfo.entity, components::JointPosition());
        if (!velComp)
          _ecm.CreateComponent(jointInfo.entity, components::JointVelocity());
      }
      continue;
    }

    // Add joint to message
    auto jointMsg = msg.add_joint();
    jointMsg->set_name(jointInfo.name);

    // Set axis1 (most joints are single-DOF)
    auto axis1 = jointMsg->mutable_axis1();

    // Position and velocity are vectors (support multi-DOF)
    const auto &positions = posComp->Data();
    const auto &velocities = velComp->Data();

    if (!positions.empty()) {
      axis1->set_position(positions[0]);
    }
    if (!velocities.empty()) {
      axis1->set_velocity(velocities[0]);
    }

    // If multi-DOF joint (rare), handle axis2
    if (positions.size() > 1 || velocities.size() > 1) {
      auto axis2 = jointMsg->mutable_axis2();
      if (positions.size() > 1) {
        axis2->set_position(positions[1]);
      }
      if (velocities.size() > 1) {
        axis2->set_velocity(velocities[1]);
      }
    }
  }

  // Publish
  auto it = this->jointPubs.find(_config.name);
  if (it != this->jointPubs.end()) {
    it->second.Publish(msg);
  }
}

//////////////////////////////////////////////////
void WorldStatePublisherPrivate::ResolveJointEntities(
    ModelConfig &_config,
    Entity _modelEntity,
    const EntityComponentManager &_ecm)
{
  _config.cached_joints.clear();

  // Name-based discovery (bypasses component tag requirements)
  // Query by name directly instead of relying on components::Joint tag
  // This is necessary because velocity-controlled joints don't have the
  // components::Joint tag visible at world-level ECS
  for (const auto &targetName : _config.joint_names) {
    // Query by name (doesn't require Joint component tag)
    Entity jointEntity = _ecm.EntityByComponents(
        components::Name(targetName));

    if (jointEntity == kNullEntity) {
      gzwarn << "WorldStatePublisher: Joint '" << targetName
             << "' not found for model '" << _config.name << "'\n";
      continue;
    }

    // Verify it belongs to our model
    auto parentComp = _ecm.Component<components::ParentEntity>(jointEntity);
    if (!parentComp) {
      gzwarn << "WorldStatePublisher: Joint '" << targetName
             << "' has no parent component\n";
      continue;
    }

    if (parentComp->Data() != _modelEntity) {
      // Joint exists but belongs to different model (name collision)
      continue;
    }

    // Cache this joint
    JointInfo info;
    info.name = targetName;
    info.entity = jointEntity;
    _config.cached_joints.push_back(info);
  }

  // Warn if not all joints found
  if (_config.cached_joints.size() != _config.joint_names.size()) {
    gzwarn << "WorldStatePublisher: Model '" << _config.name << "' found "
           << _config.cached_joints.size() << " of " << _config.joint_names.size()
           << " requested joints\n";
  }
}

//////////////////////////////////////////////////
WorldStatePublisher::WorldStatePublisher()
  : dataPtr(std::make_unique<WorldStatePublisherPrivate>())
{
}

//////////////////////////////////////////////////
void WorldStatePublisher::Configure(
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  // Verify world-level attachment
  auto world = _ecm.Component<components::World>(_entity);
  if (!world) {
    gzerr << "WorldStatePublisher must be attached to world entity\n";
    return;
  }

  this->dataPtr->worldEntity = _entity;

  // Parse global update frequency
  if (_sdf->HasElement("update_frequency")) {
    this->dataPtr->config.update_frequency =
        _sdf->Get<double>("update_frequency");
  }

  // Parse model configurations
  // Note: SDF element navigation requires non-const access
  auto sdfMutable = std::const_pointer_cast<sdf::Element>(_sdf);

  if (!sdfMutable->HasElement("model")) {
    gzwarn << "WorldStatePublisher: No models configured\n";
    return;
  }

  sdf::ElementPtr modelElem = sdfMutable->GetElement("model");
  while (modelElem) {
    ModelConfig mc;

    // Required: model name
    if (!modelElem->HasElement("name")) {
      gzerr << "Model config missing <name> element\n";
      modelElem = modelElem->GetNextElement("model");
      continue;
    }
    mc.name = modelElem->Get<std::string>("name");

    // Optional: publish flags
    if (modelElem->HasElement("publish_pose")) {
      mc.publish_pose = modelElem->Get<bool>("publish_pose");
    }
    if (modelElem->HasElement("publish_joints")) {
      mc.publish_joints = modelElem->Get<bool>("publish_joints");
    }

    // Optional: joint whitelist
    if (modelElem->HasElement("joint")) {
      sdf::ElementPtr jointElem = modelElem->GetElement("joint");
      while (jointElem) {
        mc.joint_names.push_back(jointElem->Get<std::string>());
        jointElem = jointElem->GetNextElement("joint");
      }
    }

    this->dataPtr->config.models.push_back(mc);
    modelElem = modelElem->GetNextElement("model");
  }

  gzmsg << "WorldStatePublisher configured: "
        << this->dataPtr->config.models.size() << " models, "
        << this->dataPtr->config.update_frequency << " Hz\n";
}

//////////////////////////////////////////////////
void WorldStatePublisher::PreUpdate(
    const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("WorldStatePublisher::PreUpdate");

  if (_info.paused) {
    return;
  }

  // Detect world reset: simTime jumped backwards OR iteration reset to 0
  bool resetDetected = false;
  if (_info.simTime < this->dataPtr->lastPubTime ||
      _info.iterations == 0) {
    resetDetected = true;
  }

  // Also detect reset by checking if any cached model entity is stale
  if (!resetDetected) {
    for (const auto &pair : this->dataPtr->modelCache) {
      if (!_ecm.HasEntity(pair.second)) {
        resetDetected = true;
        break;
      }
    }
  }

  if (resetDetected) {
    gzmsg << "WorldStatePublisher: Reset detected (simTime="
           << std::chrono::duration_cast<std::chrono::milliseconds>(
                  _info.simTime).count()
           << "ms, iter=" << _info.iterations
           << "), clearing caches\n";
    this->dataPtr->modelCache.clear();
    for (auto &mc : this->dataPtr->config.models) {
      mc.cached_joints.clear();
    }
    this->dataPtr->lastPubTime = std::chrono::steady_clock::duration{0};
    return;  // Skip this frame, let ECM settle after reset
  }

  // Rate limiting (global frequency)
  std::chrono::duration<double> period{1.0 / this->dataPtr->config.update_frequency};
  auto updatePeriod = std::chrono::duration_cast<
      std::chrono::steady_clock::duration>(period);

  auto diff = _info.simTime - this->dataPtr->lastPubTime;
  if (diff < updatePeriod) {
    return;
  }

  // Initialize publishers on first update
  if (!this->dataPtr->initialized) {
    this->dataPtr->Initialize(_ecm);
  }

  auto stampMsg = convert<msgs::Time>(_info.simTime);

  // Publish for each configured model
  for (auto &modelConfig : this->dataPtr->config.models) {
    // Get model entity (with caching and reset handling)
    Entity modelEntity = this->dataPtr->GetModelEntity(modelConfig.name, _ecm);
    if (modelEntity == kNullEntity) {
      continue;  // Model not found (may be during reset)
    }

    // Publish pose
    if (modelConfig.publish_pose) {
      this->dataPtr->PublishPose(modelConfig, modelEntity, _ecm, stampMsg);
    }

    // Publish joint states
    if (modelConfig.publish_joints) {
      this->dataPtr->PublishJointStates(modelConfig, modelEntity, _ecm, stampMsg);
    }
  }

  this->dataPtr->lastPubTime = _info.simTime;
}

// Register this plugin
GZ_ADD_PLUGIN(WorldStatePublisher,
              System,
              WorldStatePublisher::ISystemConfigure,
              WorldStatePublisher::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(WorldStatePublisher,
                    "gz::sim::systems::WorldStatePublisher")
