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
 * @brief Capability that runs the planning pipeline.
 */
class PlanCapability : public Capability {
 public:
  /** @brief Returns "plan". */
  std::string Name() const override { return "plan"; }

  /**
   * @brief Bind to @p server.
   * @param[in] server Runtime server (not owned).
   * @return true if @p server is non-null.
   */
  bool Init(ManipulationServer* server) override;

  /**
   * @brief Plan without executing.
   * @param[in] req Motion plan request.
   * @return Motion plan response.
   */
  ::autonomy::manipulation::proto::MotionPlanResponse Plan(
      const planner::MotionPlanRequest& req);

 private:
  ManipulationServer* server_ = nullptr;
};

}  // namespace dispatch
}  // namespace manipulation
}  // namespace autonomy
