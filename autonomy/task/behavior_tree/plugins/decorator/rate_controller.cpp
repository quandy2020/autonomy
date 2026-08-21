/*
 * Copyright 2026 The Openbot Authors
 *
 * Nav2-style RateController: tick the child at most `hz` times per second.
 * When skipped, returns SUCCESS so PipelineSequence can continue to FollowPath.
 * After the first successful child tick, later failures are soft (SUCCESS) so a
 * transient replan miss does not abort an active FollowPath.
 */

#include <chrono>
#include <string>

#include "autonomy/task/behavior_tree/plugins/bt_node_base.hpp"
#include "behaviortree_cpp/decorator_node.h"

namespace autonomy::task::plugins {

class RateController : public BT::DecoratorNode
{
public:
    RateController(const std::string& name, const BT::NodeConfig& config)
        : BT::DecoratorNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<double>("hz", 1.0, "max child tick rate")};
    }

    void halt() override
    {
        first_time_ = true;
        got_success_ = false;
        DecoratorNode::halt();
    }

private:
    std::chrono::duration<double> period_{1.0};
    std::chrono::steady_clock::time_point last_time_{};
    bool first_time_{true};
    bool got_success_{false};

    BT::NodeStatus tick() override
    {
        double hz = 1.0;
        getInput("hz", hz);
        if (hz > 1e-3) {
            period_ = std::chrono::duration<double>(1.0 / hz);
        }

        const auto now = std::chrono::steady_clock::now();
        if (!first_time_ && (now - last_time_) < period_) {
            return BT::NodeStatus::SUCCESS;
        }

        first_time_ = false;
        last_time_ = now;
        setStatus(BT::NodeStatus::RUNNING);
        const BT::NodeStatus child_status = child_node_->executeTick();
        if (child_status == BT::NodeStatus::SUCCESS) {
            got_success_ = true;
            return BT::NodeStatus::SUCCESS;
        }
        if (child_status == BT::NodeStatus::FAILURE) {
            // First plan must succeed; later replans may soft-fail.
            return got_success_ ? BT::NodeStatus::SUCCESS
                                : BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::SUCCESS;
    }
};

}  // namespace autonomy::task::plugins

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<autonomy::task::plugins::RateController>(
        "RateController");
}
