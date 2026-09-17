/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file planning_request_adapter.hpp
 * @brief Planning request adapter base (MoveIt PlanningRequestAdapter analogue).
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "autonomy/common/macros.hpp"
#include "autonomy/manipulation/common/planner_interface.hpp"

namespace autonomy {
namespace manipulation {
namespace planner {

/**
 * @brief Hook that may rewrite a request and/or post-process a response.
 *
 * Pre-plan adapters typically mutate @p request; post-plan adapters inspect or
 * rewrite @p response (e.g. time parameterization, path validation).
 */
class PlanningRequestAdapter {
 public:
  /**
   * @brief Define PlanningRequestAdapter::SharedPtr type
   */
  AUTONOMY_SMART_PTR_DEFINITIONS(PlanningRequestAdapter)

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
                     ::autonomy::manipulation::proto::MotionPlanResponse* response) const = 0;
};

}  // namespace planner
}  // namespace manipulation
}  // namespace autonomy
