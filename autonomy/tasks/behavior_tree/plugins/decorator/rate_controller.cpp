/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */
#include <chrono>

#include "autonomy/tasks/behavior_tree/bt_utils.hpp"
#include "behaviortree_cpp/decorator_node.h"
namespace autonomy {
namespace tasks {
namespace behavior_tree {

class RateController : public BT::DecoratorNode
{
public:
    RateController(const std::string& name, const BT::NodeConfiguration& conf)
        : BT::DecoratorNode(name, conf) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<double>("hz", 1.0, "Rate")};
    }

    BT::NodeStatus tick() override {
        if (!BT::isStatusActive(status())) {
            double hz = 1.0;
            getInput("hz", hz);
            period_sec_ = hz > 0.0 ? 1.0 / hz : 1.0;
            start_ = std::chrono::high_resolution_clock::now();
            first_time_ = true;
        }

        setStatus(BT::NodeStatus::RUNNING);
        const auto elapsed = std::chrono::high_resolution_clock::now() - start_;
        const double seconds =
            std::chrono::duration_cast<std::chrono::duration<double>>(elapsed)
                .count();

        if (first_time_ || child_node_->status() == BT::NodeStatus::RUNNING ||
            seconds >= period_sec_) {
            first_time_ = false;
            const BT::NodeStatus child_state = child_node_->executeTick();
            switch (child_state) {
                case BT::NodeStatus::SKIPPED:
                case BT::NodeStatus::RUNNING:
                case BT::NodeStatus::FAILURE:
                    return child_state;
                case BT::NodeStatus::SUCCESS:
                    start_ = std::chrono::high_resolution_clock::now();
                    return BT::NodeStatus::SUCCESS;
                default:
                    return BT::NodeStatus::FAILURE;
            }
        }
        return status();
    }

private:
    bool first_time_{true};
    double period_sec_{1.0};
    std::chrono::high_resolution_clock::time_point start_;
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

REGISTER_BEHAVIOR_TREE_NODE(RateController, "RateController")
