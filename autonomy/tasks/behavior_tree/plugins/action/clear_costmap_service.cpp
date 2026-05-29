/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/tasks/behavior_tree/bt_service_node.hpp"
#include "autonomy/tasks/behavior_tree/bt_utils.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class ClearEntireCostmapService : public BtServiceNode
{
public:
    ClearEntireCostmapService(const std::string& name,
                              const BT::NodeConfiguration& conf)
        : BtServiceNode(name, conf) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<std::string>("service_name", "", "Costmap id")};
    }

protected:
    void onTick() override { IncrementRecoveryCount(config()); }

    BT::NodeStatus onService() override {
        std::string service_name;
        getInput("service_name", service_name);
        auto costmap = ResolveCostmap(GetContext(config()), service_name);
        if (!costmap) {
            return BT::NodeStatus::FAILURE;
        }
        costmap->resetLayers();
        return BT::NodeStatus::SUCCESS;
    }
};

class ClearCostmapExceptRegionService : public BtServiceNode
{
public:
    ClearCostmapExceptRegionService(const std::string& name,
                                    const BT::NodeConfiguration& conf)
        : BtServiceNode(name, conf) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("service_name", "", "Costmap id"),
            BT::InputPort<double>("reset_distance", 3.0, "Keep radius m"),
        };
    }

protected:
    void onTick() override { IncrementRecoveryCount(config()); }

    BT::NodeStatus onService() override {
        std::string service_name;
        double reset_distance = 3.0;
        getInput("service_name", service_name);
        getInput("reset_distance", reset_distance);
        auto wrapper = ResolveCostmap(GetContext(config()), service_name);
        if (!wrapper) {
            return BT::NodeStatus::FAILURE;
        }
        double rx = 0.0;
        double ry = 0.0;
        if (!GetRobotMapPose(wrapper.get(), rx, ry)) {
            return BT::NodeStatus::FAILURE;
        }
        ClearCostmapByDistance(wrapper->getCostmap(), rx, ry, reset_distance,
                               true);
        return BT::NodeStatus::SUCCESS;
    }
};

class ClearCostmapAroundRobotService : public BtServiceNode
{
public:
    ClearCostmapAroundRobotService(const std::string& name,
                                   const BT::NodeConfiguration& conf)
        : BtServiceNode(name, conf) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("service_name", "", "Costmap id"),
            BT::InputPort<double>("reset_distance", 1.0, "Clear radius m"),
        };
    }

protected:
    void onTick() override { IncrementRecoveryCount(config()); }

    BT::NodeStatus onService() override {
        std::string service_name;
        double reset_distance = 1.0;
        getInput("service_name", service_name);
        getInput("reset_distance", reset_distance);
        auto wrapper = ResolveCostmap(GetContext(config()), service_name);
        if (!wrapper) {
            return BT::NodeStatus::FAILURE;
        }
        double rx = 0.0;
        double ry = 0.0;
        if (!GetRobotMapPose(wrapper.get(), rx, ry)) {
            return BT::NodeStatus::FAILURE;
        }
        ClearCostmapByDistance(wrapper->getCostmap(), rx, ry, reset_distance,
                               false);
        return BT::NodeStatus::SUCCESS;
    }
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<
        autonomy::tasks::behavior_tree::ClearEntireCostmapService>(
        "ClearEntireCostmap");
    factory.registerNodeType<
        autonomy::tasks::behavior_tree::ClearCostmapExceptRegionService>(
        "ClearCostmapExceptRegion");
    factory.registerNodeType<
        autonomy::tasks::behavior_tree::ClearCostmapAroundRobotService>(
        "ClearCostmapAroundRobot");
}
