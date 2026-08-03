/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/common/task_action_client.hpp"

#include <automsgs/actions/nav_actions.pb.h>

namespace autonomy {
namespace task {
namespace common {

namespace nav_proto = automsgs::actions;

template class TaskActionClient<nav_proto::ComputePathToPoseAction>;
template class TaskActionClient<nav_proto::ComputePathThroughPosesAction>;
template class TaskActionClient<nav_proto::SmoothPathAction>;
template class TaskActionClient<nav_proto::FollowPathAction>;
template class TaskActionClient<nav_proto::SpinAction>;
template class TaskActionClient<nav_proto::BackUpAction>;
template class TaskActionClient<nav_proto::WaitAction>;

}  // namespace common
}  // namespace task
}  // namespace autonomy
