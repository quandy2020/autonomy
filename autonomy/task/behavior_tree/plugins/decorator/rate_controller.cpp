/*
 * Copyright 2026 The Openbot Authors
 *
 * Nav2-style RateController: tick the child at most `hz` times per second so
 * ComputePath republishes /plan from the live robot pose while FollowPath runs.
 * When skipped, returns SUCCESS so PipelineSequence can continue to FollowPath.
 * After the first successful child tick, later failures are soft (SUCCESS) so a
 * transient replan miss does not abort an active FollowPath.
 * Before the first success, FAILURE is returned immediately so RecoveryNode
 * can BackUp when the robot starts inside an obstacle. RUNNING is still
 * forwarded while ComputePath is in flight.
 */

#include <chrono>
#include <string>

#include "autonomy/common/logging.hpp"
#include "autonomy/task/behavior_tree/plugins/bt_node_base.hpp"
#include "behaviortree_cpp/blackboard.h"
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
        // Recovery (backup/spin) changes the robot pose; the next pipeline
        // pass must replan instead of reusing a SUCCESS from inside a wall.
        first_time_ = true;
        got_success_ = false;
        DecoratorNode::halt();
    }

private:
    std::chrono::duration<double> period_{1.0};
    std::chrono::steady_clock::time_point last_time_{};
    bool first_time_{true};
    bool got_success_{false};
    int seen_goal_epoch_{0};

    static constexpr auto kFirstPlanRetry{std::chrono::milliseconds(400)};

    BT::NodeStatus tick() override
    {
        double hz = 1.0;
        getInput("hz", hz);
        if (hz > 1e-3) {
            period_ = std::chrono::duration<double>(1.0 / hz);
        }

        int goal_epoch = 0;
        if (config().blackboard &&
            config().blackboard->get("goal_epoch", goal_epoch) &&
            goal_epoch != seen_goal_epoch_) {
            seen_goal_epoch_ = goal_epoch;
            first_time_ = true;
            // New 2D Goal: do not keep SUCCESS from the previous path or
            // FollowPath continues tracking the old endpoint.
            got_success_ = false;
        }

        const auto now = std::chrono::steady_clock::now();
        const bool child_running =
            child_node_ && child_node_->status() == BT::NodeStatus::RUNNING;
        // hz only spaces completed replan cycles. An in-flight ComputePath
        // (RUNNING) must be ticked every BT cycle — 0.25 Hz + 3s action
        // timeout otherwise fails before the planner result is polled.
        // Before the first success, retry immediately instead of waiting
        // a full 1/hz period.
        if (!first_time_ && !child_running && got_success_ &&
            (now - last_time_) < period_) {
            return BT::NodeStatus::SUCCESS;
        }
        if (!first_time_ && !child_running && !got_success_ &&
            (now - last_time_) < kFirstPlanRetry) {
            return BT::NodeStatus::RUNNING;
        }

        setStatus(BT::NodeStatus::RUNNING);
        const BT::NodeStatus child_status = child_node_->executeTick();
        if (child_status != BT::NodeStatus::RUNNING) {
            first_time_ = false;
            last_time_ = now;
        }
        if (child_status == BT::NodeStatus::SUCCESS) {
            got_success_ = true;
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
            AWARN << "nav rate first_plan_failed";
            return BT::NodeStatus::FAILURE;
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
