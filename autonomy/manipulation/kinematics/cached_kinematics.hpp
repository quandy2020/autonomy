/*
 * Copyright 2026 The Openbot Authors
 *
 * In-memory IK cache wrapping another solver.
 */

#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "autonomy/manipulation/kinematics/kinematics_base.hpp"

namespace autonomy {
namespace manipulation {
namespace kinematics {

/**
 * @brief Decorator that caches IK solutions for repeated tip poses.
 *
 * Default constructor wraps KdlKinematics when AUTONOMY_HAS_KDL is enabled.
 */
class CachedKinematics : public KinematicsBase {
 public:
  /** @brief Construct wrapping the default inner solver (KDL when available). */
  CachedKinematics();

  /**
   * @brief Construct wrapping an explicit inner solver.
   * @param[in] inner Non-null kinematics backend.
   */
  explicit CachedKinematics(std::shared_ptr<KinematicsBase> inner);

  /**
   * @brief Forward Init to the inner solver.
   * @param[in] group Planning group name.
   * @param[in] base_frame Base frame.
   * @param[in] tip_frame Tip frame.
   * @return Inner Init result.
   */
  bool Init(const std::string& group, const std::string& base_frame,
            const std::string& tip_frame) override;

  /**
   * @brief Forward FK to the inner solver (uncached).
   * @param[in] joints Joint configuration.
   * @param[out] tip_pose Computed tip pose.
   * @return Inner FK result.
   */
  bool GetPositionFK(const core::JointState& joints,
                     Pose* tip_pose) const override;

  /**
   * @brief Return a cached IK solution or solve via the inner plugin.
   * @param[in] tip_pose Desired tip pose.
   * @param[in] seed Seed joint state (used on cache miss).
   * @param[in] options IK options (affect cache key for position_only).
   * @param[out] solution Joint solution on success.
   * @return Error code from cache hit or inner solve.
   */
  ErrorCode GetPositionIK(const Pose& tip_pose, const core::JointState& seed,
                          const IkOptions& options,
                          core::JointState* solution) const override;

  /** @brief Underlying solver used on cache miss. */
  std::shared_ptr<KinematicsBase> Inner() const { return inner_; }

 private:
  static std::string PoseKey(const Pose& pose, bool position_only);

  std::shared_ptr<KinematicsBase> inner_;
  mutable std::mutex mutex_;
  mutable std::unordered_map<std::string, core::JointState> cache_;
};

}  // namespace kinematics
}  // namespace manipulation
}  // namespace autonomy
