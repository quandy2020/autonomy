/*
 * Copyright 2026 The Openbot Authors
 *
 * Non-blocking sleep for recovery delays (no action-server dependency).
 */

#include <chrono>

#include "autonomy/task/behavior_tree/plugins/bt_node_base.hpp"

namespace autonomy::task::plugins {

class SleepAction : public BtStatefulAction
{
public:
    SleepAction(const std::string& name, const BT::NodeConfig& config)
        : BtStatefulAction(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<int>("msec", 1000, "sleep duration (ms)")};
    }

protected:
    BT::NodeStatus OnFirstTick() override
    {
        int msec = 1000;
        getInput("msec", msec);
        if (msec < 0) {
            msec = 0;
        }
        deadline_ = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(msec);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus OnExecute() override
    {
        return (std::chrono::steady_clock::now() >= deadline_)
                   ? BT::NodeStatus::SUCCESS
                   : BT::NodeStatus::RUNNING;
    }

private:
    std::chrono::steady_clock::time_point deadline_{};
};

}  // namespace autonomy::task::plugins

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<autonomy::task::plugins::SleepAction>("Sleep");
}
