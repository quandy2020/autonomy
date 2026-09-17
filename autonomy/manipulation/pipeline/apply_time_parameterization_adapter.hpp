/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file apply_time_parameterization_adapter.hpp
 * @brief Post-plan time parameterization adapter.
 */

#pragma once

#include "autonomy/manipulation/pipeline/planning_request_adapter.hpp"

namespace autonomy {
namespace manipulation {
namespace planner {

/**
 * @brief Post-plan: apply Kunz–Stilman / Ruckig time parameterization.
 *
 * Plugin aliases: "time_parameterization", "TimeParameterizeAdapter",
 * and class name "ApplyTimeParameterizationAdapter".
 */
class ApplyTimeParameterizationAdapter : public PlanningRequestAdapter {
 public:
  /**
   * @brief Config / logging id used by the planning pipeline.
   * @return Fixed string "time_parameterization".
   */
  std::string GetName() const override { return "time_parameterization"; }

  /**
   * @brief Time-parameterize @p response trajectory when planning succeeded.
   * @param[in,out] request Unused for this adapter (may be null).
   * @param[in,out] response Trajectory to stamp; left unchanged on failure.
   * @return false if parameterization fails (pipeline aborts).
   */
  bool Adapt(MotionPlanRequest* request,
             ::autonomy::manipulation::proto::MotionPlanResponse* response) const override;
};

}  // namespace planner
}  // namespace manipulation
}  // namespace autonomy
