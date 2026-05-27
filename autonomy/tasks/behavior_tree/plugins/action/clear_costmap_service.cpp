/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/tasks/behavior_tree/service_node.hpp"
#include "autonomy/tasks/behavior_tree/utils.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace {

map::costmap_2d::Costmap2DWrapper::SharedPtr ResolveCostmap(
    const std::shared_ptr<BehaviorTreeContext>& ctx,
    const std::string& service_name) {
    if (!ctx) {
        return nullptr;
    }
    if (service_name.find("global") != std::string::npos) {
        return ctx->planner ? ctx->planner->GetCostmapWrapper() : nullptr;
    }
    if (auto local = ctx->controller->GetCostmapWrapper()) {
        return local;
    }
    return ctx->planner ? ctx->planner->GetCostmapWrapper() : nullptr;
}

}  // namespace

class ClearEntireCostmapService : public ServiceNode
{
public:
    ClearEntireCostmapService(const std::string& name,
                              const BT::NodeConfiguration& conf)
        : ServiceNode(name, conf) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<std::string>("service_name", "", "Costmap id")};
    }

protected:
    void onTick() override { IncrementRecoveryCount(config()); }

    BT::NodeStatus onService() override {
        std::string service_name;
        getInput("service_name", service_name);
        auto ctx = GetContext(config());
        auto costmap = ResolveCostmap(ctx, service_name);
        if (!costmap) {
            return BT::NodeStatus::FAILURE;
        }
        costmap->resetLayers();
        return BT::NodeStatus::SUCCESS;
    }
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "autonomy/tasks/behavior_tree/node_utils.hpp"

REGISTER_BEHAVIOR_TREE_NODE(ClearEntireCostmapService, "ClearEntireCostmap")
