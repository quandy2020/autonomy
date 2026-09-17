/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/pipeline/validate_trajectory_path_adapter.hpp"

#include "autolink/plugin_manager/plugin_manager.hpp"
#include "autonomy/manipulation/motion/scene/planning_scene.hpp"
#include "autonomy/manipulation/model/error_codes.hpp"

namespace autonomy {
namespace manipulation {
namespace planner {

bool ValidateTrajectoryPathAdapter::Adapt(MotionPlanRequest* request,
                                ::autonomy::manipulation::proto::MotionPlanResponse* response) const {
  if (!request || !response || !response->success()) {
    return true;
  }
  if (request->scene &&
      !request->scene->IsPathValidDense(response->trajectory(), 4)) {
    response->set_success(false);
    response->set_error_code(ErrorCode::INVALID_MOTION_PLAN);
    response->set_error("adapter validate_path (dense) failed");
    response->mutable_trajectory()->Clear();
    return false;
  }
  return true;
}

AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(ValidateTrajectoryPathAdapter,
                                        PlanningRequestAdapter);

}  // namespace planner
}  // namespace manipulation
}  // namespace autonomy
