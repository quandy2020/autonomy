/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/common/task_action_client.hpp"

#include <automsgs/actions/nav_actions.pb.h>

namespace autonomy {
namespace task {
namespace common {

namespace navigation_actions = automsgs::actions;

template class TaskActionClient<navigation_actions::ComputePathToPoseAction>;
template class TaskActionClient<navigation_actions::ComputePathThroughPosesAction>;
template class TaskActionClient<navigation_actions::SmoothPathAction>;
template class TaskActionClient<navigation_actions::FollowPathAction>;
template class TaskActionClient<navigation_actions::SpinAction>;
template class TaskActionClient<navigation_actions::BackUpAction>;
template class TaskActionClient<navigation_actions::WaitAction>;

}  // namespace common
}  // namespace task
}  // namespace autonomy
