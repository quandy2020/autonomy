/*
 * Copyright 2026 The Openbot Authors
 *
 * BT node wrapping DockingEngine::StartDock / Tick (OpenNav DockRobot).
 */

#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include "autonomy/task/behavior_tree/plugins/bt_node_base.hpp"
#include "autonomy/task/charging/plugins/plugin_utils.hpp"

namespace autonomy::task::plugins::charging {

class DockRobotAction : public BtStatefulAction {
public:
    DockRobotAction(const std::string& name, const BT::NodeConfig& config)
        : BtStatefulAction(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("dock_id"),
            BT::InputPort<::automsgs::msgs::geometry_msgs::PoseStamped>("dock_pose"),
            BT::InputPort<std::string>("dock_type"),
            BT::InputPort<double>("max_staging_time", 1000.0, "s"),
            BT::InputPort<bool>("navigate_to_staging_pose", true, ""),
            BT::InputPort<bool>("use_dock_id", true, ""),
            BT::OutputPort<int>("error_code_id"),
            BT::OutputPort<std::string>("error_msg"),
            BT::OutputPort<int>("num_retries"),
            BT::OutputPort<bool>("success"),
        };
    }

protected:
    bool LatchAfterFirstTick() const override { return true; }

    BT::NodeStatus OnFirstTick() override {
        auto client = ResolveClient(*this);
        if (!client) {
            return BT::NodeStatus::FAILURE;
        }
        ::autonomy::task::proto::ChargingGoal goal;
        bool use_id = true;
        getInput("use_dock_id", use_id);
        std::string dock_id;
        getInput("dock_id", dock_id);
        ::automsgs::msgs::geometry_msgs::PoseStamped pose;
        getInput("dock_pose", pose);
        std::string dock_type;
        getInput("dock_type", dock_type);
        double staging = 1000.0;
        getInput("max_staging_time", staging);
        bool nav_stage = true;
        getInput("navigate_to_staging_pose", nav_stage);
        goal.set_use_dock_id(use_id);
        goal.set_dock_type(dock_type);
        goal.set_max_staging_time(static_cast<float>(staging));
        goal.set_navigate_to_staging_pose(nav_stage);
        if (use_id && !dock_id.empty()) {
            goal.set_dock_station_id(dock_id);
        } else {
            *goal.mutable_dock_pose() = pose;
        }
        client->ApplyGoal(goal);
        if (!client->StartDock()) {
            setOutput("success", false);
            setOutput("error_code_id", static_cast<int>(client->error_code()));
            setOutput("error_msg", client->error_msg());
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus OnExecute() override {
        auto client = ResolveClient(*this);
        if (!client) {
            return BT::NodeStatus::FAILURE;
        }
        const auto st = client->Tick();
        setOutput("num_retries", client->num_retries());
        using Engine = ::autonomy::task::charging::DockingEngine;
        if (st == Engine::RunState::Running) {
            return BT::NodeStatus::RUNNING;
        }
        const bool ok = st == Engine::RunState::Succeeded;
        setOutput("success", ok);
        setOutput("error_code_id", static_cast<int>(client->error_code()));
        setOutput("error_msg", client->error_msg());
        return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    void OnHalted() override {
        if (auto client = ResolveClient(*this)) {
            client->CancelActiveMotion();
        }
    }
};

}  // namespace autonomy::task::plugins::charging

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<autonomy::task::plugins::charging::DockRobotAction>(
        "DockRobot");
}
