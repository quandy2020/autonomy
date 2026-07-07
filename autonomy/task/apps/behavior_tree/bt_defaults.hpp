/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include "autonomy/commsgs/proto/vehicle_msgs.pb.h"
#include "autonomy/task/apps/behavior_tree/bt_profile.hpp"
#include "autonomy/task/proto/task_options.pb.h"

namespace autonomy {
namespace task {

/** Built-in BT paths and plugin lists (config/task/behavior_tree/…). */
class BtDefaults
{
public:
    static BtProfile ProfileFor(
        const ::autonomy::task::proto::TaskServerOptions& options,
        ::autonomy::commsgs::proto::vehicle_msgs::RobotTaskType type);

    static void Apply(::autonomy::task::proto::TaskServerOptions* options);
};

}  // namespace task
}  // namespace autonomy
