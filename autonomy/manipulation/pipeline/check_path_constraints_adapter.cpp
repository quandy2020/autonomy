/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/pipeline/check_path_constraints_adapter.hpp"

#include "autolink/plugin_manager/plugin_manager.hpp"
#include "autonomy/manipulation/constraints/constraint_samplers.hpp"

namespace autonomy {
namespace manipulation {
namespace planner {

bool CheckPathConstraintsAdapter::Adapt(MotionPlanRequest* request,
                                    ::autonomy::manipulation::proto::MotionPlanResponse* response) const {
  if (!request || !response || !response->success()) {
    return true;
  }
  if (!constraints::SatisfiesPathConstraints(*request,
                                             response->trajectory())) {
    response->set_success(false);
    response->set_error_code(ErrorCode::GOAL_VIOLATES_PATH_CONSTRAINTS);
    response->set_error("adapter check_constraints failed");
    response->mutable_trajectory()->Clear();
    return false;
  }
  return true;
}

AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(CheckPathConstraintsAdapter,
                                        PlanningRequestAdapter);

}  // namespace planner
}  // namespace manipulation
}  // namespace autonomy
