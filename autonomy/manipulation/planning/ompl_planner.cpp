/*
 * Copyright 2026 The Openbot Authors
 *
 * Real OMPL adapter — joint-space RRTConnect via SimpleSetup.
 */

#include "autonomy/manipulation/planning/ompl_planner.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include "autonomy/common/logging.hpp"
#include "autonomy/manipulation/scene/planning_scene.hpp"
#include "autolink/plugin_manager/plugin_manager.hpp"

namespace autonomy {
namespace manipulation {
namespace planning {
namespace {

namespace ob = ompl::base;
namespace og = ompl::geometric;

double BoundLo(const MotionPlanRequest& request, const std::string& name) {
  if (request.model) {
    if (const auto* lim = request.model->GetJointLimits(name)) {
      return lim->min_position;
    }
  }
  return -3.141592653589793;
}

double BoundHi(const MotionPlanRequest& request, const std::string& name) {
  if (request.model) {
    if (const auto* lim = request.model->GetJointLimits(name)) {
      return lim->max_position;
    }
  }
  return 3.141592653589793;
}

}  // namespace

bool OmplPlanner::Init(const std::string& planner_id) {
  planner_id_ = planner_id.empty() ? "ompl" : planner_id;
  AINFO << "OmplPlanner init id=" << planner_id_;
  return true;
}

MotionPlanResponse OmplPlanner::Plan(const MotionPlanRequest& request) {
  MotionPlanResponse response;
  if (request.start_state.positions.empty() ||
      request.goal_state.positions.empty() ||
      request.start_state.positions.size() !=
          request.goal_state.positions.size()) {
    response.error_code = ErrorCode::kInvalidRobotState;
    response.error = "OMPL: invalid start/goal";
    return response;
  }

  const auto names = request.goal_state.names.empty()
                         ? request.start_state.names
                         : request.goal_state.names;
  const std::size_t dof = request.start_state.positions.size();

  auto space = std::make_shared<ob::RealVectorStateSpace>(static_cast<unsigned>(dof));
  ob::RealVectorBounds bounds(static_cast<unsigned>(dof));
  for (std::size_t i = 0; i < dof; ++i) {
    const std::string n = i < names.size() ? names[i] : "";
    bounds.setLow(static_cast<unsigned>(i), BoundLo(request, n));
    bounds.setHigh(static_cast<unsigned>(i), BoundHi(request, n));
  }
  space->setBounds(bounds);

  og::SimpleSetup setup(space);
  setup.setStateValidityChecker([&](const ob::State* state) {
    const auto* rv = state->as<ob::RealVectorStateSpace::StateType>();
    core::JointState js;
    js.names = names;
    js.positions.resize(dof);
    for (std::size_t i = 0; i < dof; ++i) {
      js.positions[i] = (*rv)[static_cast<unsigned>(i)];
    }
    if (request.scene) {
      return !request.scene->CheckCollision(js);
    }
    return true;
  });

  ob::ScopedState<> start(space);
  ob::ScopedState<> goal(space);
  for (std::size_t i = 0; i < dof; ++i) {
    start[static_cast<unsigned>(i)] = request.start_state.positions[i];
    goal[static_cast<unsigned>(i)] = request.goal_state.positions[i];
  }
  setup.setStartAndGoalStates(start, goal);

  auto planner = std::make_shared<og::RRTConnect>(setup.getSpaceInformation());
  setup.setPlanner(planner);

  const double timeout =
      request.planning_time > 0.0 ? request.planning_time : 1.0;
  const ob::PlannerStatus status = setup.solve(timeout);
  if (!status) {
    response.error_code = ErrorCode::kPlanningFailed;
    response.error = "OMPL RRTConnect failed";
    return response;
  }

  setup.simplifySolution();
  og::PathGeometric path = setup.getSolutionPath();
  path.interpolate(
      std::max(2u, static_cast<unsigned int>(path.getStateCount())));

  response.trajectory.waypoints.clear();
  response.trajectory.time_from_start.clear();
  for (std::size_t i = 0; i < path.getStateCount(); ++i) {
    const auto* rv =
        path.getState(i)->as<ob::RealVectorStateSpace::StateType>();
    core::JointState js;
    js.names = names;
    js.positions.resize(dof);
    for (std::size_t j = 0; j < dof; ++j) {
      js.positions[j] = (*rv)[static_cast<unsigned>(j)];
    }
    response.trajectory.waypoints.push_back(std::move(js));
    response.trajectory.time_from_start.push_back(0.05 * static_cast<double>(i));
  }

  response.success = true;
  response.error_code = ErrorCode::kSuccess;
  return response;
}

std::shared_ptr<PlannerBase> CreateOmplPlanner() {
  return std::make_shared<OmplPlanner>();
}


AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(OmplPlanner, PlannerBase);

}  // namespace planning
}  // namespace manipulation
}  // namespace autonomy
