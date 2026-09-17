/*
 * Copyright 2026 The Openbot Authors
 *
 * Motion plan request: protobuf wire payload + runtime shared_ptrs.
 */

#pragma once

#include <memory>

#include "autonomy/manipulation/model/link_forward_kinematics.hpp"
#include "autonomy/manipulation/model/robot_model.hpp"
#include "autonomy/manipulation/proto/motion_plan.pb.h"

namespace autonomy {
namespace manipulation {

namespace common {
class CollisionInterface;
class KinematicsInterface;
}  // namespace common
namespace scene {
class PlanningScene;
}  // namespace scene

namespace planner {

/**
 * @brief Planning request: @c pb holds serializable fields; pointers are runtime.
 */
class MotionPlanRequest {
 public:
  MotionPlanRequest();

  /** @brief Wire / serializable planning fields (autonomy.manipulation.proto). */
  ::autonomy::manipulation::proto::MotionPlanRequest pb;

  std::shared_ptr<scene::PlanningScene> scene;
  std::shared_ptr<common::CollisionInterface> collision;
  std::shared_ptr<common::KinematicsInterface> kinematics;
  std::shared_ptr<model::RobotModel> model;
  /** @brief Optional FK tree for full-chain CHOMP / collision. */
  std::shared_ptr<const model::LinkForwardKinematicsTree> link_tree;
};

}  // namespace planner
}  // namespace manipulation
}  // namespace autonomy
