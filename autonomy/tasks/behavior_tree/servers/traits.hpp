/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#pragma once

#include "autonomy/tasks/proto/action/navigate_through_poses.pb.h"
#include "autonomy/tasks/proto/action/navigate_to_pose.pb.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

struct NavigateToPoseActionTraits {
    using Goal = proto::NavigateToPoseAction_Goal;
    using Feedback = proto::NavigateToPoseAction_Feedback;
    using Result = proto::NavigateToPoseAction_Result;
};

struct NavigateThroughPosesActionTraits {
    using Goal = proto::NavigateThroughPosesAction_Goal;
    using Feedback = proto::NavigateThroughPosesAction_Feedback;
    using Result = proto::NavigateThroughPosesAction_Result;
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
