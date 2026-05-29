/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/tasks/behavior_tree/bt_utils.hpp"
#include "autonomy/tasks/constants.hpp"
#include "behaviortree_cpp/condition_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace {

bool IsPathValidOnCostmap(
    const commsgs::planning_msgs::Path& path,
    const map::costmap_2d::Costmap2DWrapper::SharedPtr& costmap) {
    if (!costmap || !costmap->getCostmap()) {
        return false;
    }
    auto* map = costmap->getCostmap();
    for (const auto& pose : path.poses) {
        unsigned int mx = 0;
        unsigned int my = 0;
        if (!map->worldToMap(pose.pose.position.x, pose.pose.position.y, mx,
                             my)) {
            return false;
        }
        const unsigned char cost = map->getCost(mx, my);
        if (cost >= map::costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
            return false;
        }
    }
    return true;
}

}  // namespace

class IsPathValidCondition : public BT::ConditionNode
{
public:
    IsPathValidCondition(const std::string& name, const BT::NodeConfiguration& conf)
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

REGISTER_BEHAVIOR_TREE_NODE(IsPathValidCondition, "IsPathValid")
