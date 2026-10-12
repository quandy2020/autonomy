/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include "automanip/dispatch/capability/capability.hpp"
#include "automanip/model/error_codes.hpp"
#include "automanip/proto/motion_plan.pb.h"

namespace automanip {
namespace dispatch {

/**
 * @brief Capability that executes a precomputed trajectory.
 */
class ExecuteCapability : public Capability {
 public:
  /** @brief Returns "execute". */
  std::string Name() const override { return "execute"; }

  /**
   * @brief Bind to @p server.
   * @param[in] server Runtime server (not owned).
   * @return true if @p server is non-null.
   */
  bool Init(ManipulationServer* server) override;

  /**
   * @brief Execute @p trajectory on the server's execution manager.
   * @param[in] trajectory Joint-space waypoints.
   * @return Error code from execution.
   */
  ErrorCode Execute(
      const automsgs::msgs::trajectory_msgs::JointTrajectory& trajectory);

 private:
  ManipulationServer* server_ = nullptr;
};

}  // namespace dispatch
}  // namespace automanip
