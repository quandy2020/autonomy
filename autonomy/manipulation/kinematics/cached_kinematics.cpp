/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/kinematics/cached_kinematics.hpp"

#include <cmath>
#include <sstream>

#ifdef AUTONOMY_HAS_KDL
#include "autonomy/manipulation/kinematics/kdl_kinematics.hpp"
#else
#include "autonomy/manipulation/kinematics/stub_kinematics.hpp"
#endif

#include "autolink/plugin_manager/plugin_manager.hpp"

namespace autonomy {
namespace manipulation {
namespace kinematics {

CachedKinematics::CachedKinematics()
#ifdef AUTONOMY_HAS_KDL
    : inner_(std::make_shared<KdlKinematics>())
#else
    : inner_(std::make_shared<StubKinematics>())
#endif
{
}

CachedKinematics::CachedKinematics(std::shared_ptr<KinematicsBase> inner)
    : inner_(std::move(inner)) {}

bool CachedKinematics::Init(const std::string& group,
                            const std::string& base_frame,
                            const std::string& tip_frame) {
  return inner_ && inner_->Init(group, base_frame, tip_frame);
}

bool CachedKinematics::GetPositionFK(const core::JointState& joints,
                                     Pose* tip_pose) const {
  return inner_ && inner_->GetPositionFK(joints, tip_pose);
}

std::string CachedKinematics::PoseKey(const Pose& pose, bool position_only) {
  std::ostringstream oss;
  oss.precision(4);
  oss << pose.x << ',' << pose.y << ',' << pose.z;
  if (!position_only) {
    oss << ',' << pose.qx << ',' << pose.qy << ',' << pose.qz << ',' << pose.qw;
  }
  return oss.str();
}

ErrorCode CachedKinematics::GetPositionIK(const Pose& tip_pose,
                                          const core::JointState& seed,
                                          const IkOptions& options,
                                          core::JointState* solution) const {
  if (!inner_ || !solution) {
    return ErrorCode::kNoIkSolution;
  }
  const std::string key = PoseKey(tip_pose, options.position_only);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = cache_.find(key);
    if (it != cache_.end()) {
      *solution = it->second;
      return ErrorCode::kSuccess;
    }
  }
  const ErrorCode code =
      inner_->GetPositionIK(tip_pose, seed, options, solution);
  if (code == ErrorCode::kSuccess) {
    std::lock_guard<std::mutex> lock(mutex_);
    cache_[key] = *solution;
  }
  return code;
}

AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(CachedKinematics, KinematicsBase);

}  // namespace kinematics
}  // namespace manipulation
}  // namespace autonomy
