/*
 * Copyright 2026 The Openbot Authors
 *
 * Plan → adapters pipeline (MoveIt planning_pipeline analogue).
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "autonomy/manipulation/planning/planner_base.hpp"
#include "autonomy/manipulation/planning/planning_request_adapter.hpp"
#include "autonomy/manipulation/proto/manipulation_options.pb.h"

namespace autonomy {
namespace manipulation {
namespace planning {

/**
 * @brief Runs request adapters then the configured planner (and post-adapters).
 *
 * Mirrors MoveIt's planning_pipeline: adapters may modify the request or
 * validate / densify / time-parameterize the response.
 */
class PlanningPipeline {
 public:
  PlanningPipeline() = default;

  /**
   * @brief Load pipeline options and default adapters from proto.
   * @param[in] options Manipulation module options.
   * @return true if configured successfully.
   */
  bool Init(const proto::ManipulationOptions& options);

  /**
   * @brief Plan through adapters + planner.
   * @param[in] request Motion plan request.
   * @return Planner / adapter response.
   */
  planning::MotionPlanResponse Plan(
      const planning::MotionPlanRequest& request);

  /**
   * @brief Replace the active planner instance.
   * @param[in] planner Planner plugin (may be null until set).
   */
  void SetPlanner(std::shared_ptr<planning::PlannerBase> planner);

  /**
   * @brief Append a request / response adapter.
   * @param[in] adapter Adapter to run in registration order.
   */
  void AddAdapter(std::shared_ptr<PlanningRequestAdapter> adapter);

  /** @brief Remove all registered adapters. */
  void ClearAdapters();

 private:
  proto::ManipulationOptions options_;
  std::shared_ptr<planning::PlannerBase> planner_;
  std::vector<std::shared_ptr<PlanningRequestAdapter>> adapters_;
};

}  // namespace planning
}  // namespace manipulation
}  // namespace autonomy
