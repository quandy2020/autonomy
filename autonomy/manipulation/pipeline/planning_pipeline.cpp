/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/planner/pipeline/planning_pipeline.hpp"

#include "autonomy/common/logging.hpp"
#include "autonomy/manipulation/planner/constraint_samplers/constraint_samplers.hpp"
#include "autonomy/manipulation/common/plugin_ids.hpp"

namespace autonomy {
namespace manipulation {
namespace planning {
namespace {

const char* kDefaultAdapters[] = {
    "fix_start_state_bounds", "fix_start_state_path_constraints",
    "dense_sample", "time_parameterization", "validate_path",
    "check_constraints"};

bool IsPrePlanAdapter(const std::string& name) {
  return name == "fix_start_state_bounds" ||
         name == "FixStartStateBoundsAdapter" ||
         name == "fix_start_state_path_constraints" ||
         name == "FixStartStatePathConstraintsAdapter";
}

}  // namespace

bool PlanningPipeline::Init(const proto::ManipulationOptions& options) {
  options_ = options;
  adapters_.clear();
  RegisterManipulationPlugins();

  const auto load = [this](const std::string& name) {
    auto adapter = CreatePlugin<PlanningRequestAdapter>(name);
    if (!adapter) {
      AWARN << "PlanningPipeline: unknown adapter " << name;
      return;
    }
    adapters_.push_back(std::move(adapter));
  };

  if (options_.adapters_size() == 0) {
    for (const char* name : kDefaultAdapters) {
      load(name);
    }
  } else {
    for (const auto& name : options_.adapters()) {
      load(name);
    }
  }
  AINFO << "PlanningPipeline init group=" << options_.planning_group()
        << " planner=" << options_.planner_id()
        << " adapters=" << adapters_.size();
  return true;
}

void PlanningPipeline::SetPlanner(std::shared_ptr<PlannerBase> planner) {
  planner_ = std::move(planner);
}

void PlanningPipeline::AddAdapter(
    std::shared_ptr<PlanningRequestAdapter> adapter) {
  adapters_.push_back(std::move(adapter));
}

void PlanningPipeline::ClearAdapters() {
  adapters_.clear();
}

MotionPlanResponse PlanningPipeline::Plan(const MotionPlanRequest& request) {
  MotionPlanResponse response;
  if (!planner_) {
    response.error = "no planner";
    response.error_code = ErrorCode::kFailure;
    return response;
  }

  MotionPlanRequest req = request;

  // Pre-adapters that mutate the request run with an empty response before
  // planning; others expect a successful plan.
  MotionPlanResponse pre;
  pre.success = true;
  for (const auto& adapter : adapters_) {
    if (IsPrePlanAdapter(adapter->GetName())) {
      if (!adapter->Adapt(&req, &pre)) {
        return pre;
      }
    }
  }

  std::string cerr;
  const ErrorCode ccode =
      constraint_samplers::EvaluateRequestConstraints(req, &cerr);
  if (ccode != ErrorCode::kSuccess) {
    response.error_code = ccode;
    response.error = cerr;
    return response;
  }

  response = planner_->Plan(req);
  if (!response.success) {
    return response;
  }

  for (const auto& adapter : adapters_) {
    if (IsPrePlanAdapter(adapter->GetName())) {
      continue;
    }
    if (!adapter->Adapt(&req, &response)) {
      return response;
    }
  }
  return response;
}

}  // namespace planning
}  // namespace manipulation
}  // namespace autonomy
