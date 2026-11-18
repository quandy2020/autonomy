/*
 * Copyright 2026 The Openbot Authors
 *
 * BT node wrapping DockingEngine::StartUndock / Tick (OpenNav UndockRobot).
 */

#include "autonomy/task/behavior_tree/plugins/bt_node_base.hpp"
#include "autonomy/task/charging/plugins/plugin_utils.hpp"

namespace autonomy::task::plugins::charging {

class UndockRobotAction : public BtStatefulAction {
public:
    UndockRobotAction(const std::string& name, const BT::NodeConfig& config)
        : BtStatefulAction(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("dock_type"),
            BT::InputPort<double>("max_undocking_time", 30.0, "s"),
            BT::OutputPort<int>("error_code_id"),
            BT::OutputPort<std::string>("error_msg"),
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
        goal.set_command(::autonomy::task::proto::DOCK_CMD_UNDOCK);
        std::string dock_type;
        getInput("dock_type", dock_type);
        double timeout = 30.0;
        getInput("max_undocking_time", timeout);
        goal.set_dock_type(dock_type);
        goal.set_max_undocking_time(static_cast<float>(timeout));
        client->ApplyGoal(goal);
        if (!client->StartUndock()) {
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
    factory.registerNodeType<
        autonomy::task::plugins::charging::UndockRobotAction>("UndockRobot");
}
