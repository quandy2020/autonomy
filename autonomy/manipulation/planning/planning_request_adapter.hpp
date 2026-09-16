/*
 * Copyright 2026 The Openbot Authors
 *
 * Planning request adapters (MoveIt PlanningRequestAdapter analogue).
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "autonomy/manipulation/planning/planner_base.hpp"

namespace autonomy {
namespace manipulation {
namespace planning {

/**
 * @brief Hook that may rewrite a request and/or post-process a response.
 *
 * Pre-plan adapters typically mutate @p request; post-plan adapters inspect or
 * rewrite @p response (e.g. time parameterization, path validation).
 */
class PlanningRequestAdapter {
 public:
  virtual ~PlanningRequestAdapter() = default;

  /** @brief Human-readable adapter name for logging / config. */
  virtual std::string GetName() const = 0;

  /**
   * @brief Adapt the plan request and/or response.
   * @param[in,out] request Motion plan request (may be modified).
   * @param[in,out] response Motion plan response (may be modified).
   * @return false to abort the pipeline.
   */
  virtual bool Adapt(MotionPlanRequest* request,
                     MotionPlanResponse* response) const = 0;
};

/** @brief Post-plan: apply TOTG / Ruckig time parameterization. */
class TimeParameterizeAdapter : public PlanningRequestAdapter {
 public:
  std::string GetName() const override { return "time_parameterization"; }
  bool Adapt(MotionPlanRequest* request,
             MotionPlanResponse* response) const override;
};

/** @brief Post-plan: reject paths that collide or violate scene validity. */
class ValidatePathAdapter : public PlanningRequestAdapter {
 public:
  std::string GetName() const override { return "validate_path"; }
  bool Adapt(MotionPlanRequest* request,
             MotionPlanResponse* response) const override;
};

/**
 * @brief Post-plan: densify joint waypoints to a maximum step size.
 */
class DenseSampleAdapter : public PlanningRequestAdapter {
 public:
  /**
   * @brief Construct with maximum joint-space step.
   * @param[in] max_step Max |Δq| between consecutive waypoints.
   */
  explicit DenseSampleAdapter(double max_step = 0.1) : max_step_(max_step) {}
  std::string GetName() const override { return "dense_sample"; }
  bool Adapt(MotionPlanRequest* request,
             MotionPlanResponse* response) const override;

 private:
  double max_step_;
};

/** @brief Post-plan: verify joint path constraints along the trajectory. */
class CheckConstraintsAdapter : public PlanningRequestAdapter {
 public:
  std::string GetName() const override { return "check_constraints"; }
  bool Adapt(MotionPlanRequest* request,
             MotionPlanResponse* response) const override;
};

/** @brief Pre-plan: clamp start_state into model joint limits when model is set. */
class FixStartStateBoundsAdapter : public PlanningRequestAdapter {
 public:
  std::string GetName() const override { return "fix_start_state_bounds"; }
  bool Adapt(MotionPlanRequest* request,
             MotionPlanResponse* response) const override;
};

}  // namespace planning
}  // namespace manipulation
}  // namespace autonomy
