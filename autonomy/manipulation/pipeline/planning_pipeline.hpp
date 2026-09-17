/*
 * Copyright 2026 The Openbot Authors
 *
 * Plan → adapters pipeline (MoveIt planning_pipeline analogue).
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "autonomy/manipulation/common/planner_interface.hpp"
#include "autonomy/manipulation/pipeline/planning_request_adapter.hpp"
#include "autonomy/manipulation/proto/manipulation_options.pb.h"

namespace autonomy {
namespace manipulation {
namespace planner {

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
  ::autonomy::manipulation::proto::MotionPlanResponse Plan(
      const planner::MotionPlanRequest& request);

  /**
   * @brief Replace the active planner instance.
   * @param[in] planner Planner plugin (may be null until set).
   */
  void SetPlanner(common::PlannerInterface::SharedPtr planner);

  /**
   * @brief Append a request / response adapter.
   * @param[in] adapter Adapter to run in registration order.
   */
  void AddAdapter(std::shared_ptr<PlanningRequestAdapter> adapter);

  /** @brief Remove all registered adapters. */
  void ClearAdapters();

 private:
  proto::ManipulationOptions options_;
  common::PlannerInterface::SharedPtr planner_;
  std::vector<std::shared_ptr<PlanningRequestAdapter>> adapters_;
};

}  // namespace planner
}  // namespace manipulation
}  // namespace autonomy
