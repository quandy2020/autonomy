/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/tasks/behavior_tree/utils.hpp"
#include "autonomy/tasks/constants.hpp"
#include "behaviortree_cpp/condition_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace {

bool IsPathValidOnCostmap(
    const commsgs::planning_msgs::Path& path,
    const map::costmap_2d::Costmap2DWrapper::SharedPtr& costmap) {
    if (!costmap || path.poses.size() < kMinPathPoses) {
        return path.poses.size() >= kMinPathPoses;
    }
    auto* grid = costmap->getCostmap();
    if (!grid) {
        return true;
    }
    for (const auto& pose : path.poses) {
        unsigned int mx = 0;
        unsigned int my = 0;
        if (!grid->worldToMap(pose.pose.position.x, pose.pose.position.y, mx,
                              my)) {
            return false;
        }
        const unsigned char cost = grid->getCost(mx, my);
        using map::costmap_2d::LETHAL_OBSTACLE;
        if (cost >= LETHAL_OBSTACLE) {
            return false;
        }
    }
    return true;
}

}  // namespace

class IsPathValidCondition : public BT::ConditionNode
{
public:
    IsPathValidCondition(const std::string& name,
                         const BT::NodeConfiguration& conf)
        : BT::ConditionNode(name, conf) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<commsgs::planning_msgs::Path>("path")};
    }

    BT::NodeStatus tick() override {
        commsgs::planning_msgs::Path path;
        getInput("path", path);
        if (path.poses.empty()) {
            getInput(kBlackboardPathKey, path);
        }
        if (path.poses.size() < kMinPathPoses) {
            return BT::NodeStatus::FAILURE;
        }
        auto ctx = GetContext(config());
        auto costmap =
            ctx && ctx->planner ? ctx->planner->GetCostmapWrapper() : nullptr;
        return IsPathValidOnCostmap(path, costmap) ? BT::NodeStatus::SUCCESS
                                                   : BT::NodeStatus::FAILURE;
    }
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "autonomy/tasks/behavior_tree/node_utils.hpp"

REGISTER_BEHAVIOR_TREE_NODE(IsPathValidCondition, "IsPathValid")
