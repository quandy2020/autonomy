/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include <chrono>

#include "behaviortree_cpp/decorator_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class RateController : public BT::DecoratorNode
{
public:
    RateController(const std::string& name, const BT::NodeConfiguration& conf)
        : BT::DecoratorNode(name, conf), first_time_(false), period_(1.0) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<double>("hz", 1.0, "Rate")};
    }

    BT::NodeStatus tick() override {
        if (!BT::isStatusActive(status())) {
            getInput("hz", hz_);
            period_ = hz_ > 0.0 ? 1.0 / hz_ : 1.0;
            start_ = std::chrono::high_resolution_clock::now();
            first_time_ = true;
        }
        if (!BT::isStatusActive(status())) {
            start_ = std::chrono::high_resolution_clock::now();
            first_time_ = true;
        }
        setStatus(BT::NodeStatus::RUNNING);
        const auto now = std::chrono::high_resolution_clock::now();
        const auto elapsed = now - start_;
        const auto seconds =
            std::chrono::duration_cast<std::chrono::duration<double>>(elapsed);
        if (first_time_ ||
            child_node_->status() == BT::NodeStatus::RUNNING ||
            seconds.count() >= period_) {
            first_time_ = false;
            const BT::NodeStatus child_state = child_node_->executeTick();
            if (child_state == BT::NodeStatus::SUCCESS) {
                start_ = std::chrono::high_resolution_clock::now();
            }
            return child_state;
        }
        return status();
    }

private:
    bool first_time_;
    double period_;
    double hz_{1.0};
    std::chrono::high_resolution_clock::time_point start_;
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<autonomy::tasks::behavior_tree::RateController>(
        "RateController");
}
