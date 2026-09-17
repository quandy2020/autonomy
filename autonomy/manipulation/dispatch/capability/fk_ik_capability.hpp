/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include "autonomy/manipulation/dispatch/capability/capability.hpp"
#include "autonomy/manipulation/model/error_codes.hpp"
#include "autonomy/manipulation/common/kinematics_interface.hpp"

namespace autonomy {
namespace manipulation {
namespace dispatch {

/**
 * @brief Capability for forward / inverse kinematics.
 */
class FkIkCapability : public Capability {
 public:
  /** @brief Returns "fk_ik". */
  std::string Name() const override { return "fk_ik"; }

  /**
   * @brief Bind to @p server.
   * @param[in] server Runtime server (not owned).
   * @return true if @p server is non-null.
   */
  bool Init(ManipulationServer* server) override;

  /**
   * @brief Compute FK for @p joints into @p pose.
   * @param[in] joints Joint configuration.
   * @param[out] pose Resulting end-effector pose.
   * @return true on success.
   */
  bool ComputeFk(const automsgs::msgs::sensor_msgs::JointState& joints,
                 automsgs::msgs::geometry_msgs::Pose* pose) const;

  /**
   * @brief Compute IK for @p pose seeded by @p seed.
   * @param[in] pose Desired end-effector pose.
   * @param[in] seed Seed joint state.
   * @param[out] solution Joint solution on success.
   * @return Error code from the kinematics plugin.
   */
  ErrorCode ComputeIk(const automsgs::msgs::geometry_msgs::Pose& pose,
                      const automsgs::msgs::sensor_msgs::JointState& seed,
                      automsgs::msgs::sensor_msgs::JointState* solution) const;

 private:
  ManipulationServer* server_ = nullptr;
};

}  // namespace dispatch
}  // namespace manipulation
}  // namespace autonomy
