/*
 * Copyright 2026 The Openbot Authors
 *
 * In-memory IK cache wrapping another solver (LRU via autonomy::common::LRUCache).
 */

#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "autonomy/common/lru_cache.hpp"
#include "autonomy/manipulation/common/kinematics_interface.hpp"

namespace autonomy {
namespace manipulation {
namespace kinematics {

/**
 * @brief Decorator that caches IK solutions for repeated tip poses.
 *
 * Uses @ref autonomy::common::LRUCache. Default wraps KdlKinematics when
 * AUTONOMY_HAS_KDL is enabled.
 */
class CachedKinematics : public KinematicsBase {
 public:
  /** @brief Construct wrapping the default inner solver (KDL when available). */
  CachedKinematics();

  /**
   * @brief Construct wrapping an explicit inner solver.
   * @param[in] inner Non-null kinematics backend.
   * @param[in] cache_size LRU capacity.
   */
  explicit CachedKinematics(std::shared_ptr<KinematicsBase> inner,
                            std::size_t cache_size = 256);

  bool Init(const std::string& group, const std::string& base_frame,
            const std::string& tip_frame) override;

  bool GetPositionFK(const core::JointState& joints,
                     Pose* tip_pose) const override;

  ErrorCode GetPositionIK(const Pose& tip_pose, const core::JointState& seed,
                          const IkOptions& options,
                          core::JointState* solution) const override;

  /** @brief Underlying solver used on cache miss. */
  std::shared_ptr<KinematicsBase> Inner() const { return inner_; }

  /** @brief Drop all cached solutions. */
  void ClearCache() const;

  /** @brief Current number of cached entries. */
  std::size_t CacheSize() const;

 private:
  static std::string BuildPoseCacheKey(const Pose& pose, bool position_only);

  std::shared_ptr<KinematicsBase> inner_;
  mutable std::mutex mutex_;
  mutable common::LRUCache<std::string, core::JointState> cache_;
};

}  // namespace kinematics
}  // namespace manipulation
}  // namespace autonomy
