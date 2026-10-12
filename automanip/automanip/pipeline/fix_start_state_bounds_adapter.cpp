/*
 * Copyright 2026 The Openbot Authors
 */

#include "automanip/pipeline/fix_start_state_bounds_adapter.hpp"

#include <algorithm>

#include "autolink/plugin_manager/plugin_manager.hpp"
#include "automanip/constraints/constraint_samplers.hpp"

namespace automanip {
namespace planner {

bool FixStartStateBoundsAdapter::Adapt(
    MotionPlanRequest* request, ::automanip::proto::MotionPlanResponse* /*response*/) const {
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
}  // namespace automanip
