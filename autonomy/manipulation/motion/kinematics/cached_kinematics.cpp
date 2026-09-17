/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/motion/kinematics/cached_kinematics.hpp"

#include <sstream>
#include <utility>

#ifdef AUTONOMY_HAS_KDL
#include "autonomy/manipulation/motion/kinematics/kdl_kinematics.hpp"
#else
#include "autonomy/manipulation/motion/kinematics/null_kinematics.hpp"
#endif

#include "autolink/plugin_manager/plugin_manager.hpp"

namespace autonomy {
namespace manipulation {
namespace kinematics {

CachedKinematics::CachedKinematics()
#ifdef AUTONOMY_HAS_KDL
    : inner_(std::make_shared<KdlKinematics>()), cache_(256)
#else
    : inner_(std::make_shared<NullKinematics>()), cache_(256)
#endif
{
}

CachedKinematics::CachedKinematics(KinematicsInterface::SharedPtr inner,
                                   std::size_t cache_size)
    : inner_(std::move(inner)), cache_(cache_size > 0 ? cache_size : 1) {}

bool CachedKinematics::Init(const std::string& group,
                            const std::string& base_frame,
                            const std::string& tip_frame) {
  return inner_ && inner_->Init(group, base_frame, tip_frame);
}

bool CachedKinematics::GetPositionFK(const automsgs::msgs::sensor_msgs::JointState& joints,
                                     automsgs::msgs::geometry_msgs::Pose* tip_pose) const {
  return inner_ && inner_->GetPositionFK(joints, tip_pose);
}

void CachedKinematics::ClearCache() const {
  std::lock_guard<std::mutex> lock(mutex_);
  cache_.Clear();
}

std::size_t CachedKinematics::CacheSize() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return cache_.size();
}

std::string CachedKinematics::BuildPoseCacheKey(const automsgs::msgs::geometry_msgs::Pose& pose,
                                                bool position_only) {
  std::ostringstream oss;
  oss.precision(4);
  oss << pose.position().x() << ',' << pose.position().y() << ','
      << pose.position().z();
  if (!position_only) {
    oss << ',' << pose.orientation().x() << ',' << pose.orientation().y()
        << ',' << pose.orientation().z() << ',' << pose.orientation().w();
  }
  return oss.str();
}

ErrorCode CachedKinematics::GetPositionIK(const automsgs::msgs::geometry_msgs::Pose& tip_pose,
                                          const automsgs::msgs::sensor_msgs::JointState& seed,
                                          const InverseKinematicsOptions& options,
                                          automsgs::msgs::sensor_msgs::JointState* solution) const {
  if (!inner_ || !solution) {
    return ErrorCode::NO_INVERSE_KINEMATICS_SOLUTION;
  }
  const std::string key = BuildPoseCacheKey(tip_pose, options.position_only());
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (automsgs::msgs::sensor_msgs::JointState* hit = cache_.GetSilently(key)) {
      *solution = *hit;
      return ErrorCode::SUCCESS;
    }
  }
  const ErrorCode code =
      inner_->GetPositionIK(tip_pose, seed, options, solution);
  if (code == ErrorCode::SUCCESS) {
    std::lock_guard<std::mutex> lock(mutex_);
    cache_.Put(key, *solution);
  }
  return code;
}

AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(CachedKinematics, KinematicsInterface);

}  // namespace kinematics
}  // namespace manipulation
}  // namespace autonomy
