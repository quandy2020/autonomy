/*
 * Copyright 2026 The Openbot Authors
 *
 * Builtin TaskInterface plugin registration (composition helper for TaskServer).
 */

#ifndef AUTONOMY_TASK_REGISTER_TASKS_HPP_
#define AUTONOMY_TASK_REGISTER_TASKS_HPP_

#include <memory>
#include <type_traits>
#include <utility>

#include "autonomy/task/charging/charging.hpp"
#include "autonomy/task/exploration/exploration.hpp"
#include "autonomy/task/localization/localization.hpp"
#include "autonomy/task/mapping/mapping.hpp"
#include "autonomy/task/navigation/navigation.hpp"
#include "autonomy/task/teleop/teleop.hpp"
#include "autonomy/task/tracking/tracking.hpp"
#include "autonomy/task/proto/task_options.pb.h"

namespace autonomy {
namespace task {

/**
 * Creates enabled tasks and passes each to @p register_task.
 * Typed out-parameters receive the live handles for TaskServer accessors.
 */
template <typename RegisterApp>
void RegisterBuiltinTasks(
    const proto::TaskAppOptions& apps, RegisterApp&& register_app,
    NavigationTask::SharedPtr* navigation, TrackerTask::SharedPtr* tracking,
    TeleopTask::SharedPtr* teleop, ExplorationTask::SharedPtr* exploration,
    ChargingTask::SharedPtr* charging, MappingTask::SharedPtr* mapping,
    LocalizationTask::SharedPtr* localization)
{
    const auto add = [&](auto* out, bool enable) {
        if (!enable || out == nullptr) {
            return;
        }
        using SharedPtr = std::remove_pointer_t<decltype(out)>;
        using TaskT = typename SharedPtr::element_type;
        *out = std::make_shared<TaskT>();
        register_app(*out);
    };

    add(navigation, apps.enable_navigation());
    add(tracking, apps.enable_tracking());
    add(teleop, apps.enable_teleop());
    add(exploration, apps.enable_exploration());
    add(charging, apps.enable_charging());
    add(mapping, apps.enable_mapping());
    add(localization, apps.enable_localization());
}

}  // namespace task
}  // namespace autonomy

#endif  // AUTONOMY_TASK_REGISTER_TASKS_HPP_
