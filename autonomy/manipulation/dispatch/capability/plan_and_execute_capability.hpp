/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include "autonomy/manipulation/dispatch/capability/capability.hpp"
#include "autonomy/manipulation/common/planner_interface.hpp"

namespace autonomy {
namespace manipulation {
namespace dispatch {

/**
 * @brief Capability that plans then executes in one call.
 */
class PlanAndExecuteCapability : public Capability {
 public:
  /** @brief Returns "plan_and_execute". */
  std::string Name() const override { return "plan_and_execute"; }

  /**
   * @brief Bind to @p server.
   * @param[in] server Runtime server (not owned).
   * @return true if @p server is non-null.
   */
  bool Init(ManipulationServer* server) override;

  /**
   * @brief Plan @p req and execute the resulting trajectory on success.
   * @param[in] req Motion plan request.
   * @return Motion plan response (includes execution outcome when applicable).
   */
  ::autonomy::manipulation::proto::MotionPlanResponse Run(
      const planner::MotionPlanRequest& req);

 private:
  ManipulationServer* server_ = nullptr;
};

}  // namespace dispatch
}  // namespace manipulation
}  // namespace autonomy
