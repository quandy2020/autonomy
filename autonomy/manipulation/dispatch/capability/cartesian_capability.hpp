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
 * @brief Capability for Cartesian-path planning.
 */
class CartesianCapability : public Capability {
 public:
  /** @brief Returns "cartesian". */
  std::string Name() const override { return "cartesian"; }

  /**
   * @brief Bind to @p server.
   * @param[in] server Runtime server (not owned).
   * @return true if @p server is non-null.
   */
  bool Init(ManipulationServer* server) override;

  /**
   * @brief Plan a Cartesian path for @p req.
   * @param[in] req Motion plan request with Cartesian goals / constraints.
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
