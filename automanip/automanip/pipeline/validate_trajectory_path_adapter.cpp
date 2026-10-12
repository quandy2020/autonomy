/*
 * Copyright 2026 The Openbot Authors
 */

#include "automanip/pipeline/validate_trajectory_path_adapter.hpp"

#include "autolink/plugin_manager/plugin_manager.hpp"
#include "automanip/motion/scene/planning_scene.hpp"
#include "automanip/model/error_codes.hpp"

namespace automanip {
namespace planner {

bool ValidateTrajectoryPathAdapter::Adapt(MotionPlanRequest* request,
                                ::automanip::proto::MotionPlanResponse* response) const {
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
}  // namespace automanip
