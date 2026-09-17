/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/pipeline/planning_pipeline.hpp"

#include "autonomy/common/logging.hpp"
#include "autonomy/manipulation/constraints/constraint_samplers.hpp"
#include "autonomy/manipulation/plugin_ids.hpp"

namespace autonomy {
namespace manipulation {
namespace planner {
namespace {

const char* kDefaultAdapters[] = {
    "fix_start_state_bounds", "fix_start_state_path_constraints",
    "dense_sample", "time_parameterization", "validate_path",
    "check_constraints"};

bool IsPrePlanningAdapter(const std::string& name) {
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

void PlanningPipeline::SetPlanner(PlannerInterface::SharedPtr planner) {
  planner_ = std::move(planner);
}

void PlanningPipeline::AddAdapter(
    std::shared_ptr<PlanningRequestAdapter> adapter) {
  adapters_.push_back(std::move(adapter));
}

void PlanningPipeline::ClearAdapters() {
  adapters_.clear();
}

::autonomy::manipulation::proto::MotionPlanResponse PlanningPipeline::Plan(const MotionPlanRequest& request) {
  ::autonomy::manipulation::proto::MotionPlanResponse response;
  if (!planner_) {
    response.set_error("no planner");
    response.set_error_code(ErrorCode::FAILURE);
    return response;
  }

  MotionPlanRequest req = request;

  // Pre-adapters that mutate the request run with an empty response before
  // planning; others expect a successful plan.
  ::autonomy::manipulation::proto::MotionPlanResponse pre;
  pre.set_success(true);
  for (const auto& adapter : adapters_) {
    if (IsPrePlanningAdapter(adapter->GetName())) {
      if (!adapter->Adapt(&req, &pre)) {
        return pre;
      }
    }
  }

  std::string cerr;
  const ErrorCode ccode =
      constraints::EvaluateRequestConstraints(req, &cerr);
  if (ccode != ErrorCode::SUCCESS) {
    response.set_error_code(ccode);
    response.set_error(cerr);
    return response;
  }

  response = planner_->Plan(req);
  if (!response.success()) {
    return response;
  }

  for (const auto& adapter : adapters_) {
    if (IsPrePlanningAdapter(adapter->GetName())) {
      continue;
    }
    if (!adapter->Adapt(&req, &response)) {
      return response;
    }
  }
  return response;
}

}  // namespace planner
}  // namespace manipulation
}  // namespace autonomy
