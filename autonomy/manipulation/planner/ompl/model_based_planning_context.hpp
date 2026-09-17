/*
 * Copyright 2026 The Openbot Authors
 *
 * ModelBasedPlanningContext lite (MoveIt ompl_interface subset).
 */

#pragma once

#include <memory>
#include <string>

#include "autonomy/manipulation/constraints/constraint_sampler_manager.hpp"
#include "autonomy/manipulation/planner/ompl/ompl_planning_config.hpp"
#include "autonomy/manipulation/common/planner_interface.hpp"

namespace autonomy {
namespace manipulation {
namespace planner {

/**
 * @brief Per-request OMPL planning context (MoveIt ModelBasedPlanningContext lite).
 *
 * Full MoveIt owns ModelBasedStateSpace + SimpleSetup lifecycle here. This lite
 * applies PlannerConfig, runs constraint projection, then delegates solve to
 * an injected PlannerInterface (typically OmplPlanner).
 */
class ModelBasedPlanningContext {
 public:
  void SetPlanner(common::PlannerInterface::SharedPtr planner) {
    planner_ = std::move(planner);
  }

  void SetConstraintSamplerManager(
      constraints::ConstraintSamplerManager* mgr) {
    sampler_manager_ = mgr;
  }

  void SetConfig(OmplPlannerConfig config) { config_ = std::move(config); }

  const OmplPlannerConfig& GetConfig() const { return config_; }

  /**
   * @brief Apply config onto request + project start; then planner_->Plan.
   */
  ::autonomy::manipulation::proto::MotionPlanResponse Solve(const MotionPlanRequest& request) {
    MotionPlanRequest req = request;
    UseConfig(&req);
    if (sampler_manager_ &&
        (!(req.pb.position_constraints_size() == 0) ||
         !(req.pb.orientation_constraints_size() == 0) ||
         !(req.pb.joint_constraints_size() == 0)) &&
        req.pb.start_state().position_size() > 0) {
      automsgs::msgs::sensor_msgs::JointState s = req.pb.start_state();
      if (sampler_manager_->Project(req, &s)) {
        *req.pb.mutable_start_state() = std::move(s);
      }
    }
    if (!planner_) {
      ::autonomy::manipulation::proto::MotionPlanResponse r;
      r.set_error("ModelBasedPlanningContext: no planner");
      return r;
    }
    const std::string id =
        req.pb.planner_id().empty()
            ? (config_.planner_id.empty() ? "ompl" : config_.planner_id)
            : req.pb.planner_id();
    planner_->Init(id);
    ::autonomy::manipulation::proto::MotionPlanResponse resp = planner_->Plan(req);
    if (resp.success() && simplify_solution_ && resp.trajectory().points_size() > 2) {
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
        (req->pb.planner_id().empty() || req->pb.planner_id() == config_.name ||
         req->pb.planner_id().find('[') != std::string::npos)) {
      req->pb.set_planner_id(config_.planner_id);
    }
    if (config_.planning_time > 0) {
      req->pb.set_planning_time(config_.planning_time);
    }
    if (config_.max_attempts > 0) {
      req->pb.set_max_attempts(config_.max_attempts);
    }
  }

  common::PlannerInterface::SharedPtr planner_;
  constraints::ConstraintSamplerManager* sampler_manager_ = nullptr;
  OmplPlannerConfig config_;
  bool simplify_solution_ = true;
  bool interpolate_solution_ = true;
};

}  // namespace planner
}  // namespace manipulation
}  // namespace autonomy
