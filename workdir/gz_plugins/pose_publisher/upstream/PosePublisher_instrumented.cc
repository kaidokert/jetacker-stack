/*
 * INSTRUMENTED VERSION - Added verbose logging to debug reset behavior
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 */

#include "PosePublisher.hh"

#include <gz/msgs/pose.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/time.pb.h>

#include <stack>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <sdf/Joint.hh>

#include <gz/common/Profiler.hh>
#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/Util.hh"
#include "gz/sim/components/CanonicalLink.hh"
#include "gz/sim/components/ChildLinkName.hh"
#include "gz/sim/components/Collision.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/JointType.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/ParentLinkName.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/Sensor.hh"
#include "gz/sim/components/Visual.hh"
#include "gz/sim/Conversions.hh"
#include "gz/sim/Model.hh"

using namespace gz;
using namespace sim;
using namespace systems;

/// \brief Private data class for PosePublisher
class gz::sim::systems::PosePublisherPrivate
{
  public: void InitializeEntitiesToPublish(const EntityComponentManager &_ecm);
  public: void FillPoses(const EntityComponentManager &_ecm,
    std::vector<std::pair<Entity, math::Pose3d>> &_poses,
    bool _static);
  public: void PublishPoses(
    std::vector<std::pair<Entity, math::Pose3d>> &_poses,
    const msgs::Time &_stampMsg,
    transport::Node::Publisher &_publisher);

  public: transport::Node node;
  public: transport::Node::Publisher posePub;
  public: bool staticPosePublisher = false;
  public: transport::Node::Publisher poseStaticPub;
  public: Model model{kNullEntity};
  public: bool publishLinkPose = true;
  public: bool publishVisualPose = false;
  public: bool publishCollisionPose = false;
  public: bool publishSensorPose = false;
  public: bool publishNestedModelPose = false;
  public: bool publishModelPose = false;
  public: double updateFrequency = -1;
  public: std::chrono::steady_clock::duration lastPosePubTime{0};
  public: std::chrono::steady_clock::duration lastStaticPosePubTime{0};
  public: std::chrono::steady_clock::duration updatePeriod{0};
  public: std::chrono::steady_clock::duration staticUpdatePeriod{0};
  public: std::unordered_map<Entity, std::pair<std::string, std::string>>
    entitiesToPublish;
  public: std::unordered_set<Entity> dynamicEntities;
  public: std::vector<std::pair<Entity, math::Pose3d>> poses;
  public: std::vector<std::pair<Entity, math::Pose3d>> staticPoses;
  public: msgs::Pose poseMsg;
  public: msgs::Pose_V poseVMsg;
  public: bool usePoseV = false;
  public: bool initialized{false};

  // INSTRUMENTATION: Track PostUpdate calls
  public: uint64_t postUpdateCallCount{0};
  public: uint64_t configureCallCount{0};
};

//////////////////////////////////////////////////
PosePublisher::PosePublisher()
  : dataPtr(std::make_unique<PosePublisherPrivate>())
{
  gzmsg << "[INSTRUMENT] PosePublisher: Constructor called" << std::endl;
}

//////////////////////////////////////////////////
void PosePublisher::Configure(const Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  EntityComponentManager &_ecm,
  EventManager &/*_eventMgr*/)
{
  this->dataPtr->configureCallCount++;
  gzmsg << "[INSTRUMENT] PosePublisher: Configure called (count="
        << this->dataPtr->configureCallCount << ", entity=" << _entity << ")"
        << std::endl;

  this->dataPtr->model = Model(_entity);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "PosePublisher plugin should be attached to a model entity. "
      << "Failed to initialize." << std::endl;
    return;
  }

  auto modelName = this->dataPtr->model.Name(_ecm);
  gzmsg << "[INSTRUMENT] PosePublisher: Attached to model '" << modelName
        << "' (entity=" << _entity << ")" << std::endl;

  // parse optional params
  this->dataPtr->publishLinkPose = _sdf->Get<bool>("publish_link_pose",
    this->dataPtr->publishLinkPose).first;

  this->dataPtr->publishNestedModelPose =
    _sdf->Get<bool>("publish_nested_model_pose",
    this->dataPtr->publishNestedModelPose).first;

  this->dataPtr->publishModelPose =
    _sdf->Get<bool>("publish_model_pose",
    this->dataPtr->publishNestedModelPose).first;

  this->dataPtr->publishVisualPose =
    _sdf->Get<bool>("publish_visual_pose",
    this->dataPtr->publishVisualPose).first;

  this->dataPtr->publishCollisionPose =
    _sdf->Get<bool>("publish_collision_pose",
    this->dataPtr->publishCollisionPose).first;

  this->dataPtr->publishSensorPose =
    _sdf->Get<bool>("publish_sensor_pose",
    this->dataPtr->publishSensorPose).first;

  double updateFrequency = _sdf->Get<double>("update_frequency", -1).first;

  if (updateFrequency > 0)
  {
    std::chrono::duration<double> period{1 / updateFrequency};
    this->dataPtr->updatePeriod =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
  }

  this->dataPtr->staticPosePublisher =
    _sdf->Get<bool>("static_publisher",
    this->dataPtr->staticPosePublisher).first;

  if (this->dataPtr->staticPosePublisher)
  {
    double staticPoseUpdateFrequency =
      _sdf->Get<double>("static_update_frequency", updateFrequency).first;

    if (staticPoseUpdateFrequency > 0)
    {
      std::chrono::duration<double> period{1 / staticPoseUpdateFrequency};
      this->dataPtr->staticUpdatePeriod =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        period);
    }
  }

  // create publishers
  this->dataPtr->usePoseV =
    _sdf->Get<bool>("use_pose_vector_msg", this->dataPtr->usePoseV).first;

  std::string poseTopic = topicFromScopedName(_entity, _ecm, false) + "/pose";
  if (poseTopic.empty())
  {
    poseTopic = "/pose";
    gzerr << "Empty pose topic generated for pose_publisher system. "
      << "Setting to " << poseTopic << std::endl;
  }
  std::string staticPoseTopic = poseTopic + "_static";

  gzmsg << "[INSTRUMENT] PosePublisher: Publishing to topic: " << poseTopic
        << std::endl;

  if (this->dataPtr->usePoseV)
  {
    this->dataPtr->posePub =
      this->dataPtr->node.Advertise<msgs::Pose_V>(poseTopic);

    if (this->dataPtr->staticPosePublisher)
    {
      this->dataPtr->poseStaticPub =
        this->dataPtr->node.Advertise<msgs::Pose_V>(
        staticPoseTopic);
    }
  }
  else
  {
    this->dataPtr->posePub =
      this->dataPtr->node.Advertise<msgs::Pose>(poseTopic);
    if (this->dataPtr->staticPosePublisher)
    {
      this->dataPtr->poseStaticPub =
        this->dataPtr->node.Advertise<msgs::Pose>(
        staticPoseTopic);
    }
  }

  gzmsg << "[INSTRUMENT] PosePublisher: Configure complete" << std::endl;
}

//////////////////////////////////////////////////
void PosePublisher::PostUpdate(const UpdateInfo &_info,
  const EntityComponentManager &_ecm)
{
  GZ_PROFILE("PosePublisher::PostUpdate");

  this->dataPtr->postUpdateCallCount++;

  // Log every 100 calls or first 5 calls
  if (this->dataPtr->postUpdateCallCount <= 5 ||
      this->dataPtr->postUpdateCallCount % 100 == 0)
  {
    gzmsg << "[INSTRUMENT] PosePublisher: PostUpdate called (count="
          << this->dataPtr->postUpdateCallCount
          << ", simTime=" << std::chrono::duration<double>(_info.simTime).count()
          << "s, paused=" << _info.paused
          << ", modelValid=" << this->dataPtr->model.Valid(_ecm)
          << ", modelEntity=" << this->dataPtr->model.Entity()
          << ")" << std::endl;
  }

  // Check if model is still valid
  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzwarn << "[INSTRUMENT] PosePublisher: Model is NO LONGER VALID! "
           << "(entity=" << this->dataPtr->model.Entity()
           << ", postUpdateCount=" << this->dataPtr->postUpdateCallCount
           << ")" << std::endl;
    return;
  }

  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "[INSTRUMENT] Detected jump back in time ["
      << std::chrono::duration<double>(_info.dt).count()
      << "s]. System may not work properly." << std::endl;
  }

  if (_info.paused)
    return;

  bool publish = true;
  auto diff = _info.simTime - this->dataPtr->lastPosePubTime;
  if ((diff > std::chrono::steady_clock::duration::zero()) &&
    (diff < this->dataPtr->updatePeriod))
  {
    publish = false;
  }

  bool publishStatic = true;
  auto staticDiff = _info.simTime - this->dataPtr->lastStaticPosePubTime;
  if (!this->dataPtr->staticPosePublisher ||
    ((staticDiff > std::chrono::steady_clock::duration::zero()) &&
    (staticDiff < this->dataPtr->staticUpdatePeriod)))
  {
    publishStatic = false;
  }

  if (!publish && !publishStatic)
    return;

  if (!this->dataPtr->initialized)
  {
    gzmsg << "[INSTRUMENT] PosePublisher: Initializing entities to publish..."
          << std::endl;
    this->dataPtr->InitializeEntitiesToPublish(_ecm);
    this->dataPtr->initialized = true;
    gzmsg << "[INSTRUMENT] PosePublisher: Found "
          << this->dataPtr->entitiesToPublish.size()
          << " entities to publish" << std::endl;
  }

  if (this->dataPtr->staticPosePublisher)
  {
    if (publishStatic)
    {
      this->dataPtr->staticPoses.clear();
      this->dataPtr->FillPoses(_ecm, this->dataPtr->staticPoses, true);
      this->dataPtr->PublishPoses(this->dataPtr->staticPoses,
        convert<msgs::Time>(_info.simTime), this->dataPtr->poseStaticPub);
      this->dataPtr->lastStaticPosePubTime = _info.simTime;
    }

    if (publish)
    {
      this->dataPtr->poses.clear();
      this->dataPtr->FillPoses(_ecm, this->dataPtr->poses, false);
      this->dataPtr->PublishPoses(this->dataPtr->poses,
        convert<msgs::Time>(_info.simTime), this->dataPtr->posePub);
      this->dataPtr->lastPosePubTime = _info.simTime;
    }
  }
  else if (publish)
  {
    this->dataPtr->poses.clear();
    this->dataPtr->FillPoses(_ecm, this->dataPtr->poses, true);
    this->dataPtr->FillPoses(_ecm, this->dataPtr->poses, false);
    this->dataPtr->PublishPoses(this->dataPtr->poses,
      convert<msgs::Time>(_info.simTime), this->dataPtr->posePub);
    this->dataPtr->lastPosePubTime = _info.simTime;

    // Log publishing every 50 times
    if (this->dataPtr->postUpdateCallCount % 50 == 0)
    {
      gzmsg << "[INSTRUMENT] PosePublisher: Published "
            << this->dataPtr->poses.size() << " poses" << std::endl;
    }
  }
}

//////////////////////////////////////////////////
void PosePublisherPrivate::InitializeEntitiesToPublish(
  const EntityComponentManager &_ecm)
{
  std::stack<Entity> toCheck;
  toCheck.push(this->model.Entity());
  std::vector<Entity> visited;
  while (!toCheck.empty())
  {
    Entity entity = toCheck.top();
    toCheck.pop();
    visited.push_back(entity);

    auto link = _ecm.Component<components::Link>(entity);
    auto visual = _ecm.Component<components::Visual>(entity);
    auto collision = _ecm.Component<components::Collision>(entity);
    auto sensor = _ecm.Component<components::Sensor>(entity);
    auto joint = _ecm.Component<components::Joint>(entity);

    auto isModel = _ecm.Component<components::Model>(entity);
    auto parent = _ecm.Component<components::ParentEntity>(entity);

    bool fillPose = (link && this->publishLinkPose) ||
      (visual && this->publishVisualPose) ||
      (collision && this->publishCollisionPose) ||
      (sensor && this->publishSensorPose);

    if (isModel)
    {
      if (parent)
      {
        auto nestedModel = _ecm.Component<components::Model>(parent->Data());
        if (nestedModel)
          fillPose = this->publishNestedModelPose;
        else
          fillPose = this->publishModelPose;
      }
    }

    if (fillPose)
    {
      std::string frame;
      std::string childFrame;
      auto entityName = _ecm.Component<components::Name>(entity);
      if (!entityName)
        continue;
      childFrame =
        removeParentScope(scopedName(entity, _ecm, "::", false), "::");

      if (parent)
      {
        auto parentName = _ecm.Component<components::Name>(parent->Data());
        if (parentName)
        {
          frame = removeParentScope(
            scopedName(parent->Data(), _ecm, "::", false), "::");
        }
      }
      this->entitiesToPublish[entity] = std::make_pair(frame, childFrame);
    }

    if (this->staticPosePublisher && joint)
    {
      sdf::JointType jointType =
        _ecm.Component<components::JointType>(entity)->Data();
      if (jointType != sdf::JointType::INVALID &&
        jointType != sdf::JointType::FIXED)
      {
        std::string parentLinkName =
          _ecm.Component<components::ParentLinkName>(entity)->Data();
        std::string childLinkName =
          _ecm.Component<components::ChildLinkName>(entity)->Data();

        auto parentLinkEntity = _ecm.EntityByComponents(
          components::Name(parentLinkName), components::Link(),
          components::ParentEntity(this->model.Entity()));
        auto childLinkEntity = _ecm.EntityByComponents(
          components::Name(childLinkName), components::Link(),
          components::ParentEntity(this->model.Entity()));

        if (!_ecm.Component<components::CanonicalLink>(parentLinkEntity))
          this->dynamicEntities.insert(parentLinkEntity);
        if (!_ecm.Component<components::CanonicalLink>(childLinkEntity))
          this->dynamicEntities.insert(childLinkEntity);
      }
    }

    auto childEntities =
      _ecm.ChildrenByComponents(entity, components::ParentEntity(entity));

    for (auto childIt = childEntities.rbegin(); childIt != childEntities.rend();
      ++childIt)
    {
      auto it = std::find(visited.begin(), visited.end(), *childIt);
      if (it == visited.end())
      {
        toCheck.push(*childIt);
      }
    }
  }

  for (auto const &ent : this->dynamicEntities)
  {
    if (this->entitiesToPublish.find(ent) == this->entitiesToPublish.end())
    {
      gzwarn << "Entity id: '" << ent << "' not found when creating a list "
        << "of dynamic entities in pose publisher." << std::endl;
    }
  }

  if (this->staticPosePublisher)
  {
    this->poses.reserve(this->dynamicEntities.size());
    this->staticPoses.reserve(
      this->entitiesToPublish.size() - this->dynamicEntities.size());
  }
  else
  {
    this->poses.reserve(this->entitiesToPublish.size());
  }
}

//////////////////////////////////////////////////
void PosePublisherPrivate::FillPoses(const EntityComponentManager &_ecm,
  std::vector<std::pair<Entity, math::Pose3d>> &_poses, bool _static)
{
  GZ_PROFILE("PosePublisher::FillPose");

  for (const auto &entity : this->entitiesToPublish)
  {
    auto pose = _ecm.Component<components::Pose>(entity.first);
    if (!pose)
      continue;

    bool isStatic = this->dynamicEntities.find(entity.first) ==
      this->dynamicEntities.end();

    if (_static == isStatic)
      _poses.emplace_back(entity.first, pose->Data());
  }
}

//////////////////////////////////////////////////
void PosePublisherPrivate::PublishPoses(
  std::vector<std::pair<Entity, math::Pose3d>> &_poses,
  const msgs::Time &_stampMsg,
  transport::Node::Publisher &_publisher)
{
  GZ_PROFILE("PosePublisher::PublishPoses");

  msgs::Pose *msg = nullptr;
  if (this->usePoseV)
    this->poseVMsg.Clear();

  for (const auto &[entity, pose] : _poses)
  {
    auto entityIt = this->entitiesToPublish.find(entity);
    if (entityIt == this->entitiesToPublish.end())
      continue;

    if (this->usePoseV)
    {
      msg = this->poseVMsg.add_pose();
    }
    else
    {
      this->poseMsg.Clear();
      msg = &this->poseMsg;
    }

    GZ_ASSERT(msg != nullptr, "Pose msg is null");
    auto header = msg->mutable_header();

    header->mutable_stamp()->CopyFrom(_stampMsg);
    const std::string &frameId = entityIt->second.first;
    const std::string &childFrameId = entityIt->second.second;
    const math::Pose3d &transform = pose;
    auto frame = header->add_data();
    frame->set_key("frame_id");
    frame->add_value(frameId);
    auto childFrame = header->add_data();
    childFrame->set_key("child_frame_id");
    childFrame->add_value(childFrameId);

    msg->set_name(childFrameId);
    msgs::Set(msg, transform);

    if (!this->usePoseV)
      _publisher.Publish(this->poseMsg);
  }

  if (this->usePoseV)
    _publisher.Publish(this->poseVMsg);
}

GZ_ADD_PLUGIN(PosePublisher,
  System,
  PosePublisher::ISystemConfigure,
  PosePublisher::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(PosePublisher,
  "gz::sim::systems::PosePublisher")
