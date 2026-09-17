/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/pipeline/apply_time_parameterization_adapter.hpp"

#include <algorithm>
#include <cmath>

#include "autolink/plugin_manager/plugin_manager.hpp"
#include "autonomy/manipulation/pipeline/time_parameterization.hpp"

namespace autonomy {
namespace manipulation {
namespace planner {

bool ApplyTimeParameterizationAdapter::Adapt(
    MotionPlanRequest* request, ::autonomy::manipulation::proto::MotionPlanResponse* response) const {
  if (!request || !response || !response->success()) {
    return true;
  }
  trajectory::TimeParameterizationOptions options;
  options.set_max_velocity(request->pb.max_velocity());
  options.set_max_acceleration(request->pb.max_acceleration());
  options.set_path_tolerance(std::max(1e-3, request->pb.blend_radius()));
  if (request->model && response->trajectory().points_size() > 0) {
    const int joint_count = response->trajectory().joint_names_size();
    options.mutable_max_velocity_vector()->Clear();
    options.mutable_max_acceleration_vector()->Clear();
    options.mutable_max_velocity_vector()->Resize(joint_count, 0.0);
    options.mutable_max_acceleration_vector()->Resize(joint_count, 0.0);
    for (int i = 0; i < joint_count; ++i) {
      const auto* lim = request->model->GetJointLimits(
          response->trajectory().joint_names(i));
      options.set_max_velocity_vector(
          i, lim && lim->max_velocity > 0
                 ? lim->max_velocity *
                       std::max(1e-3, request->pb.velocity_scale())
                 : options.max_velocity() *
                       std::max(1e-3, request->pb.velocity_scale()));
      options.set_max_acceleration_vector(
          i, lim && lim->max_acceleration > 0 ? lim->max_acceleration
                                              : options.max_acceleration());
    }
  }
#ifdef AUTONOMY_HAS_RUCKIG
  if (trajectory::ApplyRuckigTrajectorySmoothing(response->mutable_trajectory(), options)) {
    return true;
  }
#endif
  return trajectory::ApplyTimeOptimalTrajectoryGeneration(response->mutable_trajectory(), options);
}

AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(ApplyTimeParameterizationAdapter,
                                        PlanningRequestAdapter);

}  // namespace planner
}  // namespace manipulation
}  // namespace autonomy
