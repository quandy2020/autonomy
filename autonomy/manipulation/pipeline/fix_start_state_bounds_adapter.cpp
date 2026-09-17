/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/pipeline/fix_start_state_bounds_adapter.hpp"

#include <algorithm>

#include "autolink/plugin_manager/plugin_manager.hpp"
#include "autonomy/manipulation/constraints/constraint_samplers.hpp"

namespace autonomy {
namespace manipulation {
namespace planner {

bool FixStartStateBoundsAdapter::Adapt(
    MotionPlanRequest* request, ::autonomy::manipulation::proto::MotionPlanResponse* /*response*/) const {
  if (!request) {
    return true;
  }
  if (request->model) {
    auto* start = request->pb.mutable_start_state();
    for (int i = 0; i < start->name_size(); ++i) {
      if (i >= start->position_size()) {
        break;
      }
      const auto* lim = request->model->GetJointLimits(start->name(i));
      if (!lim || !lim->has_position_limits()) {
        continue;
      }
      start->set_position(
          i, std::clamp(start->position(i), lim->min_position(),
                        lim->max_position()));
    }
  }
  if (request->pb.joint_constraints_size() > 0) {
    constraints::ClampToJointConstraints(*request,
                                         request->pb.mutable_start_state());
  }
  return true;
}

AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(FixStartStateBoundsAdapter,
                                        PlanningRequestAdapter);

}  // namespace planner
}  // namespace manipulation
}  // namespace autonomy
