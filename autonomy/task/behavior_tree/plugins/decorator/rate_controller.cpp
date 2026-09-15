/*
 * Copyright 2026 The Openbot Authors
 *
 * Nav2-style RateController: tick the child at most `hz` times per second.
 * When skipped, returns SUCCESS so PipelineSequence can continue to FollowPath.
 * After the first successful child tick, later failures are soft (SUCCESS) so a
 * transient replan miss does not abort an active FollowPath.
 * Before the first success, FAILURE/RUNNING keep the decorator RUNNING so a
 * cold-start planner miss does not trip RecoveryNode immediately.
 */

#include <chrono>
#include <string>

#include "autonomy/common/logging.hpp"
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
        // Keep got_success_ across RecoveryNode retries so a transient replan
        // miss after clear does not immediately FAIL the whole pipeline.
        first_time_ = true;
        waiting_first_plan_ = false;
        DecoratorNode::halt();
    }

private:
    std::chrono::duration<double> period_{1.0};
    std::chrono::steady_clock::time_point last_time_{};
    std::chrono::steady_clock::time_point first_wait_start_{};
    bool first_time_{true};
    bool got_success_{false};
    bool waiting_first_plan_{false};

    static constexpr std::chrono::seconds kFirstPlanTimeout{30};

    BT::NodeStatus tick() override
    {
        double hz = 1.0;
        getInput("hz", hz);
        if (hz > 1e-3) {
            period_ = std::chrono::duration<double>(1.0 / hz);
        }

        const auto now = std::chrono::steady_clock::now();
        if (!first_time_ && (now - last_time_) < period_) {
            // Between replan ticks: pipeline must stay alive so FollowPath
            // keeps running. If we still have no first plan, stay RUNNING.
            return got_success_ ? BT::NodeStatus::SUCCESS
                                : BT::NodeStatus::RUNNING;
        }

        first_time_ = false;
        last_time_ = now;
        setStatus(BT::NodeStatus::RUNNING);
        const BT::NodeStatus child_status = child_node_->executeTick();
        if (child_status == BT::NodeStatus::SUCCESS) {
            got_success_ = true;
            waiting_first_plan_ = false;
            return BT::NodeStatus::SUCCESS;
        }
        if (child_status == BT::NodeStatus::RUNNING) {
            return BT::NodeStatus::RUNNING;
        }
        if (child_status == BT::NodeStatus::FAILURE) {
            // Soft-fail replans once we already have a path for FollowPath.
            if (got_success_) {
                return BT::NodeStatus::SUCCESS;
            }
            if (!waiting_first_plan_) {
                waiting_first_plan_ = true;
                first_wait_start_ = now;
            }
            if (now - first_wait_start_ >= kFirstPlanTimeout) {
                AWARN << "RateController '" << name()
                      << "': first plan timed out after "
                      << kFirstPlanTimeout.count() << "s";
                waiting_first_plan_ = false;
                return BT::NodeStatus::FAILURE;
            }
            AWARN_EVERY(25) << "RateController '" << name()
                            << "': waiting for first successful plan";
            return BT::NodeStatus::RUNNING;
        }
        return got_success_ ? BT::NodeStatus::SUCCESS
                            : BT::NodeStatus::RUNNING;
    }
};

}  // namespace autonomy::task::plugins

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<autonomy::task::plugins::RateController>(
        "RateController");
}
