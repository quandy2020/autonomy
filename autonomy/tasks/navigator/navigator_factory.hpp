/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "autonomy/common/factory.hpp"
#include "autonomy/control/utils/odometry_utils.hpp"
#include "autonomy/tasks/common/feedback_utils.hpp"
#include "autonomy/tasks/common/task_context.hpp"
#include "autonomy/tasks/proto/task_options.pb.h"

namespace autonomy {
namespace tasks {
namespace common {
class NavigatorBase;
class NavigatorMuxer;
}  // namespace common

namespace navigator {

/** Inputs shared by all BT navigator creators. */
struct NavigatorCreateContext {
    const proto::TaskOptions& options;
    std::shared_ptr<common::TaskContext> task_context;
    std::vector<std::string> plugin_lib_names;
    common::FeedbackUtils feedback_utils;
    std::shared_ptr<common::NavigatorMuxer> muxer;
    std::shared_ptr<control::utils::OdomSmoother> odom_smoother;
};

using NavigatorCreator = std::unique_ptr<common::NavigatorBase> (*)(
    const NavigatorCreateContext&, const proto::NavigatorConfig&);

using NavigatorFactoryRegistry =
    ::autonomy::common::Factory<std::string, common::NavigatorBase,
                                NavigatorCreator>;

/**
 * @brief Process-wide BT navigator factory (string id → navigator instance).
 *
 * Built-in ids: navigate_to_pose, navigate_through_poses, navigate_to_docking,
 * track_to_target, explore_to_anywhere.
 */
class NavigatorFactory
{
public:
    static NavigatorFactoryRegistry& RegistryInstance();

    static void EnsureBuiltinsRegistered();

    static bool HasNavigator(const std::string& id);

    static std::unique_ptr<common::NavigatorBase> Create(
        const std::string& id, const NavigatorCreateContext& context,
        const proto::NavigatorConfig& config);

    static bool Register(const std::string& id, NavigatorCreator creator) {
        return RegistryInstance().Register(id, creator);
    }
};

bool RegisterBuiltinNavigators(NavigatorFactoryRegistry& factory);

}  // namespace navigator
}  // namespace tasks
}  // namespace autonomy
