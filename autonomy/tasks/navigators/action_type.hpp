/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#pragma once

#include "autonomy/commsgs/task_msgs.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

namespace task_proto {
using namespace commsgs::proto::task_msgs;
}  // namespace task_proto

struct NavigateToPoseActionTraits {
    using Goal = task_proto::NavigateToPoseAction_Goal;
    using Feedback = task_proto::NavigateToPoseAction_Feedback;
    using Result = task_proto::NavigateToPoseAction_Result;
};

struct NavigateThroughPosesActionTraits {
    using Goal = task_proto::NavigateThroughPosesAction_Goal;
    using Feedback = task_proto::NavigateThroughPosesAction_Feedback;
    using Result = task_proto::NavigateThroughPosesAction_Result;
};

struct FollowPathActionTraits {
    using Goal = task_proto::FollowPathAction_Goal;
    using Feedback = task_proto::FollowPathAction_Feedback;
    using Result = task_proto::FollowPathAction_Result;
};

struct ComputePathToPoseActionTraits {
    using Goal = task_proto::ComputePathToPoseAction_Goal;
    using Feedback = task_proto::ComputePathToPoseAction_Feedback;
    using Result = task_proto::ComputePathToPoseAction_Result;
};

struct ComputePathThroughPosesActionTraits {
    using Goal = task_proto::ComputePathThroughPosesAction_Goal;
    using Feedback = task_proto::ComputePathThroughPosesAction_Feedback;
    using Result = task_proto::ComputePathThroughPosesAction_Result;
};

struct SmoothPathActionTraits {
    using Goal = task_proto::SmoothPathAction_Goal;
    using Feedback = task_proto::SmoothPathAction_Feedback;
    using Result = task_proto::SmoothPathAction_Result;
};

struct SpinActionTraits {
    using Goal = task_proto::SpinAction_Goal;
    using Feedback = task_proto::SpinAction_Feedback;
    using Result = task_proto::SpinAction_Result;
};

struct BackUpActionTraits {
    using Goal = task_proto::BackUpAction_Goal;
    using Feedback = task_proto::BackUpAction_Feedback;
    using Result = task_proto::BackUpAction_Result;
};

struct DriveOnHeadingActionTraits {
    using Goal = task_proto::DriveOnHeadingAction_Goal;
    using Feedback = task_proto::DriveOnHeadingAction_Feedback;
    using Result = task_proto::DriveOnHeadingAction_Result;
};

struct WaitActionTraits {
    using Goal = task_proto::WaitAction_Goal;
    using Feedback = task_proto::WaitAction_Feedback;
    using Result = task_proto::WaitAction_Result;
};

/** Autolink action server names (nav2 uses the same strings in each plugin). */
constexpr char kFollowPathActionName[] = "follow_path";
constexpr char kComputePathToPoseActionName[] = "compute_path_to_pose";
constexpr char kComputePathThroughPosesActionName[] = "compute_path_through_poses";
constexpr char kSmoothPathActionName[] = "smooth_path";
constexpr char kSpinActionName[] = "spin";
constexpr char kBackUpActionName[] = "backup";
constexpr char kDriveOnHeadingActionName[] = "drive_on_heading";
constexpr char kWaitActionName[] = "wait";

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
