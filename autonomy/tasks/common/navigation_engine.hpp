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

#include "autonomy/common/macros.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/tasks/proto/task_options.pb.h"

namespace autolink {
class Node;
}

namespace autonomy {
namespace control {
class ControllerServer;
}
namespace planning {
class PlannerServer;
class SmootherServer;
}
namespace transform {
class Buffer;
}
namespace tasks {
namespace common {

/** Active navigation session kind (mirrors registered BT navigators). */
enum class NavigationMode {
    NONE,
    NAVIGATE_TO_POSE,
    NAVIGATE_THROUGH_POSES,
};

/**
 * @brief Backend for behavior-tree navigation (BtNavigator implementation plugs in here).
 */
class NavigationEngine
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(NavigationEngine)

    virtual ~NavigationEngine() = default;

    virtual bool Configure(
        const proto::TaskOptions& options,
        std::shared_ptr<planning::PlannerServer> planner,
        std::shared_ptr<planning::SmootherServer> smoother,
        std::shared_ptr<control::ControllerServer> controller,
        std::shared_ptr<transform::Buffer> tf_buffer) = 0;

    virtual bool StartNavigateToPose(
        const commsgs::geometry_msgs::PoseStamped& goal,
        const std::string& behavior_tree_file) = 0;

    virtual bool StartNavigateThroughPoses(
        const std::vector<commsgs::geometry_msgs::PoseStamped>& goals,
        const std::string& behavior_tree_file) = 0;

    virtual bool Cancel() = 0;
    virtual bool Pause() = 0;
    virtual bool Resume() = 0;

    virtual NavigationMode GetActiveMode() const = 0;
    virtual bool IsActive() const = 0;

    /** Optional: expose navigate_to_pose / navigate_through_poses autolink actions. */
    virtual bool AttachAutolinkNode(std::shared_ptr<autolink::Node> node) {
        (void)node;
        return true;
    }
};

/** No-op engine used until BtNavigator is wired; Configure succeeds, starts fail. */
std::shared_ptr<NavigationEngine> CreateStubNavigationEngine();

/** Behavior-tree navigator backend (BtNavigator). */
std::shared_ptr<NavigationEngine> CreateBtNavigationEngine();

}  // namespace common
}  // namespace tasks
}  // namespace autonomy
