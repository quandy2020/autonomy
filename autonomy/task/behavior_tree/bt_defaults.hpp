/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <automsgs/msgs/vehicle_msgs/robot_task_type.pb.h>
#include "autonomy/task/behavior_tree/bt_profile.hpp"
#include <automsgs/task/task_options.pb.h>

namespace autonomy {
namespace task {

/** Built-in BT paths and plugin lists (config/task/behavior_tree/…). */
class BtDefaults
{
public:
    static BtProfile ProfileFor(
        const ::autonomy::task::proto::TaskServerOptions& options,
        ::automsgs::msgs::vehicle_msgs::RobotTaskType type);

    static void Apply(::autonomy::task::proto::TaskServerOptions* options);
};

}  // namespace task
}  // namespace autonomy
