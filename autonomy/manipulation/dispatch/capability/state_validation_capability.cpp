/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/dispatch/capability/state_validation_capability.hpp"

#include "autolink/plugin_manager/plugin_manager.hpp"
#include "autonomy/manipulation/constraints/constraint_samplers.hpp"
#include "autonomy/manipulation/model/error_codes.hpp"
#include "autonomy/manipulation/manipulation_server.hpp"

namespace autonomy {
namespace manipulation {
namespace dispatch {

bool StateValidationCapability::Init(ManipulationServer* server) {
  server_ = server;
  return server_ != nullptr;
}

StateValidationResult StateValidationCapability::CheckState(
    const automsgs::msgs::sensor_msgs::JointState& state,
    const planner::MotionPlanRequest& constraints) const {
  StateValidationResult out;
  if (!server_ || !server_->scene()) {
    out.set_error_code(ErrorCode::INVALID_PLANNING_SCENE);
    out.set_error("no planning scene");
    return out;
  }
  if (!server_->scene()->IsStateValid(state)) {
    const auto info = server_->scene()->CheckCollisionDetailed(state);
    out.set_error_code(ErrorCode::START_STATE_IN_COLLISION);
    out.set_error("state in collision");
    out.set_contact_body_a(info.contact_body_a());
    out.set_contact_body_b(info.contact_body_b());
    return out;
  }
  const planner::MotionPlanRequest& req = constraints;
  if (!constraints::SatisfiesJointConstraints(req, state)) {
    out.set_error_code(ErrorCode::START_STATE_VIOLATES_PATH_CONSTRAINTS);
    out.set_error("state violates joint constraints");
    return out;
  }
  auto* kin = server_->kinematics();
  if (kin &&
      (!(req.pb.position_constraints_size() == 0) ||
       !(req.pb.orientation_constraints_size() == 0))) {
    automsgs::msgs::geometry_msgs::Pose tip;
    if (kin->GetPositionFK(state, &tip)) {
      for (const auto& c : req.pb.position_constraints()) {
        if (!constraints::SatisfiesPositionConstraint(c, tip)) {
          out.set_error_code(ErrorCode::START_STATE_VIOLATES_PATH_CONSTRAINTS);
          out.set_error("state violates position constraint");
          return out;
        }
      }
      for (const auto& c : req.pb.orientation_constraints()) {
        if (!constraints::SatisfiesOrientationConstraint(c, tip)) {
          out.set_error_code(ErrorCode::START_STATE_VIOLATES_PATH_CONSTRAINTS);
          out.set_error("state violates orientation constraint");
          return out;
        }
      }
    }
  }
  out.set_valid(true);
  out.set_error_code(ErrorCode::SUCCESS);
  return out;
}

std::vector<StateValidationResult> StateValidationCapability::CheckStates(
    const std::vector<automsgs::msgs::sensor_msgs::JointState>& states,
    const planner::MotionPlanRequest& constraints) const {
  std::vector<StateValidationResult> results;
  results.reserve(states.size());
  for (const auto& s : states) {
    results.push_back(CheckState(s, constraints));
  }
  return results;
}

AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(StateValidationCapability, Capability);

}  // namespace dispatch
}  // namespace manipulation
}  // namespace autonomy
