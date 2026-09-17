/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "autonomy/manipulation/dispatch/capability/capability.hpp"

namespace autonomy {
namespace manipulation {
namespace dispatch {

/**
 * @brief Capability to query planners and get/set runtime planner params.
 */
class QueryPlannersCapability : public Capability {
 public:
  std::string Name() const override { return "query_planners"; }
  bool Init(ManipulationServer* server) override;

  /** @brief Registered planner aliases / class names. */
  std::vector<PlannerInterfaceInfo> ListPlanners() const;

  /**
   * @brief Read planner runtime params from ManipulationOptions.
   * @param[in] planner_id Planner id (empty = active options.planner_id).
   * @return Key/value params (planner_id, planning_time_milliseconds, …).
   */
  std::unordered_map<std::string, std::string> GetPlannerParams(
      const std::string& planner_id = "") const;

  /**
   * @brief Update runtime options for subsequent plans.
   * @param[in] planner_id Target planner id written into options when non-empty.
   * @param[in] params Supported keys: planner_id, planning_time_milliseconds, max_attempts,
   *                   max_velocity, max_acceleration.
   * @return true if at least one param applied.
   */
  bool SetPlannerParams(const std::string& planner_id,
                        const std::unordered_map<std::string, std::string>& params);

 private:
  ManipulationServer* server_ = nullptr;
};

}  // namespace dispatch
}  // namespace manipulation
}  // namespace autonomy
