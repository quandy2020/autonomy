//
// Created by xiang on 25-7-7.
//

#ifndef LIGHTNING_TIMED_POSE_H
#define LIGHTNING_TIMED_POSE_H

#include "autonomy/localization/atlas/port/common/eigen_types.hpp"

namespace atlas_lio {

/// 带时间的pose
struct TimedPose {
    TimedPose() {}
    TimedPose(double t, const SE3& pose) : timestamp_(t), pose_(pose) {}
    double timestamp_ = 0;
    SE3 pose_;

    bool operator<(const TimedPose& timed_pose) const { return ((timestamp_ - timed_pose.timestamp_) < 0); };
};

}  // namespace atlas_lio

#endif  // LIGHTNING_TIMED_POSE_H
