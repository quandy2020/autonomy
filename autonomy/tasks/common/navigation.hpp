/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

enum class NavigationMode {
    NONE,
    NAVIGATE_TO_POSE,
    NAVIGATE_THROUGH_POSES,
};

/** Pluggable backend for Task navigation (BT engine or future implementations). */
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

    virtual bool AttachAutolinkNode(std::shared_ptr<autolink::Node> node) {
        (void)node;
        return true;
    }
};

std::shared_ptr<NavigationEngine> CreateBehaviorTreeNavigationEngine();

}  // namespace tasks
}  // namespace autonomy
