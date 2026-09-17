/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/pipeline/fix_start_state_path_constraints_adapter.hpp"

#include "autolink/plugin_manager/plugin_manager.hpp"
#include "autonomy/manipulation/constraints/constraint_samplers.hpp"
#include "autonomy/manipulation/pipeline/fix_start_state_bounds_adapter.hpp"

namespace autonomy {
namespace manipulation {
namespace planner {

bool FixStartStatePathConstraintsAdapter::Adapt(
    MotionPlanRequest* request, ::autonomy::manipulation::proto::MotionPlanResponse* response) const {
  if (!request) {
    return true;
  }
  // Bounds + joint bands first.
  FixStartStateBoundsAdapter bounds;
  bounds.Adapt(request, response);

  const bool has_cart = request->pb.position_constraints_size() > 0 ||
                        request->pb.orientation_constraints_size() > 0;
  if (!has_cart && request->pb.joint_constraints_size() == 0) {
    return true;
  }
  if (has_cart) {
    if (!constraints::ProjectOntoCartesianConstraints(
            *request, request->pb.mutable_start_state())) {
      if (response) {
        response->set_success(false);
        response->set_error_code(ErrorCode::START_STATE_VIOLATES_PATH_CONSTRAINTS);
        response->set_error(
            "fix_start_state_path_constraints: Cartesian project failed");
      }
      return false;
    }
  } else if (!constraints::ClampToJointConstraints(
                 *request, request->pb.mutable_start_state())) {
    if (response) {
      response->set_success(false);
      response->set_error_code(ErrorCode::START_STATE_VIOLATES_PATH_CONSTRAINTS);
      response->set_error(
          "fix_start_state_path_constraints: joint clamp failed");
    }
    return false;
  }
  return true;
}

AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(FixStartStatePathConstraintsAdapter,
                                        PlanningRequestAdapter);

}  // namespace planner
}  // namespace manipulation
}  // namespace autonomy
