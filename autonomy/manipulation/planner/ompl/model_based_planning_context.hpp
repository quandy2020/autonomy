/*
 * Copyright 2026 The Openbot Authors
 *
 * ModelBasedPlanningContext lite (MoveIt ompl_interface subset).
 */

#pragma once

#include <memory>
#include <string>

#include "autonomy/manipulation/planner/constraint_samplers/constraint_sampler_manager.hpp"
#include "autonomy/manipulation/planner/ompl/ompl_planning_config.hpp"
#include "autonomy/manipulation/common/planner_interface.hpp"

namespace autonomy {
namespace manipulation {
namespace planning {

/**
 * @brief Per-request OMPL planning context (MoveIt ModelBasedPlanningContext lite).
 *
 * Full MoveIt owns ModelBasedStateSpace + SimpleSetup lifecycle here. This lite
 * applies PlannerConfig, runs constraint projection, then delegates solve to
 * an injected PlannerBase (typically OmplPlanner).
 */
class ModelBasedPlanningContext {
 public:
  void SetPlanner(std::shared_ptr<PlannerBase> planner) {
    planner_ = std::move(planner);
  }

  void SetConstraintSamplerManager(
      constraint_samplers::ConstraintSamplerManager* mgr) {
    sampler_manager_ = mgr;
  }

  void SetConfig(OmplPlannerConfig config) { config_ = std::move(config); }

  const OmplPlannerConfig& GetConfig() const { return config_; }

  /**
   * @brief Apply config onto request + project start; then planner_->Plan.
   */
  MotionPlanResponse Solve(const MotionPlanRequest& request) {
    MotionPlanRequest req = request;
    UseConfig(&req);
    if (sampler_manager_ &&
        (!req.position_constraints.empty() ||
         !req.orientation_constraints.empty() ||
         !req.joint_constraints.empty()) &&
        req.start_state.position_size() > 0) {
      core::JointState s = req.start_state;
      if (sampler_manager_->Project(req, &s)) {
        req.start_state = std::move(s);
      }
    }
    if (!planner_) {
      MotionPlanResponse r;
      r.error = "ModelBasedPlanningContext: no planner";
      return r;
    }
    const std::string id =
        req.planner_id.empty()
            ? (config_.planner_id.empty() ? "ompl" : config_.planner_id)
            : req.planner_id;
    planner_->Init(id);
    MotionPlanResponse resp = planner_->Plan(req);
    if (resp.success && simplify_solution_ && resp.trajectory.points_size() > 2) {
      // Path simplification is done inside OmplPlanner; hook reserved.
      (void)interpolate_solution_;
    }
    return resp;
  }

  void SetSimplifySolution(bool v) { simplify_solution_ = v; }
  void SetInterpolateSolution(bool v) { interpolate_solution_ = v; }

 private:
  void UseConfig(MotionPlanRequest* req) const {
    if (!req) {
      return;
    }
    if (!config_.planner_id.empty() &&
        (req->planner_id.empty() || req->planner_id == config_.name ||
         req->planner_id.find('[') != std::string::npos)) {
      req->planner_id = config_.planner_id;
    }
    if (config_.planning_time > 0) {
      req->planning_time = config_.planning_time;
    }
    if (config_.max_attempts > 0) {
      req->max_attempts = config_.max_attempts;
    }
  }

  std::shared_ptr<PlannerBase> planner_;
  constraint_samplers::ConstraintSamplerManager* sampler_manager_ = nullptr;
  OmplPlannerConfig config_;
  bool simplify_solution_ = true;
  bool interpolate_solution_ = true;
};

}  // namespace planning
}  // namespace manipulation
}  // namespace autonomy
