/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include <algorithm>
#include <chrono>
#include <cmath>

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/tasks/behavior_tree/bt_utils.hpp"
#include "behaviortree_cpp/decorator_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class SpeedController : public BT::DecoratorNode
{
public:
    SpeedController(const std::string& name, const BT::NodeConfiguration& conf)
        : BT::DecoratorNode(name, conf), first_tick_(true) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("min_rate", 0.1, "Hz"),
            BT::InputPort<double>("max_rate", 1.0, "Hz"),
            BT::InputPort<double>("min_speed", 0.0, "m/s"),
            BT::InputPort<double>("max_speed", 0.5, "m/s"),
            BT::InputPort<commsgs::geometry_msgs::PoseStamped>("goal"),
            BT::InputPort<std::vector<commsgs::geometry_msgs::PoseStamped>>(
                "goals"),
        };
    }

    BT::NodeStatus tick() override {
        if (!BT::isStatusActive(status())) {
            getInput("min_rate", min_rate_);
            getInput("max_rate", max_rate_);
            getInput("min_speed", min_speed_);
            getInput("max_speed", max_speed_);
            d_rate_ = max_rate_ - min_rate_;
            d_speed_ = max_speed_ - min_speed_;
            if (min_rate_ <= 0.0 || max_rate_ <= 0.0) {
                return BT::NodeStatus::FAILURE;
            }
            GetInputOrBlackboard(*this, config(), "goal", kBlackboardGoalKey, goal_);
            GetInputOrBlackboard(*this, config(), "goals", kBlackboardGoalsKey,
                                 goals_);
            period_sec_ = 1.0 / max_rate_;
            start_ = std::chrono::steady_clock::now();
            first_tick_ = true;
        }

        commsgs::geometry_msgs::PoseStamped current_goal;
        std::vector<commsgs::geometry_msgs::PoseStamped> current_goals;
        GetInputOrBlackboard(*this, config(), "goal", kBlackboardGoalKey,
                             current_goal);
        GetInputOrBlackboard(*this, config(), "goals", kBlackboardGoalsKey,
                             current_goals);
        if (!PoseStampedEqual(goal_, current_goal) ||
            !GoalsEqual(goals_, current_goals)) {
            goal_ = current_goal;
            goals_ = current_goals;
            period_sec_ = 1.0 / max_rate_;
            start_ = std::chrono::steady_clock::now();
            first_tick_ = true;
        }

        setStatus(BT::NodeStatus::RUNNING);
        const auto elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start_);
        if (first_tick_ || child_node_->status() == BT::NodeStatus::RUNNING ||
            elapsed.count() >= period_sec_) {
            first_tick_ = false;
            if (elapsed.count() >= period_sec_) {
                UpdatePeriod();
                start_ = std::chrono::steady_clock::now();
            }
            return child_node_->executeTick();
        }
        return status();
    }

private:
    double GetScaledRate(double speed) const {
        if (d_speed_ <= 1e-9) {
            return max_rate_;
        }
        return std::max(
            std::min(((speed - min_speed_) / d_speed_) * d_rate_ + min_rate_,
                     max_rate_),
            min_rate_);
    }

    void UpdatePeriod() {
        auto ctx = GetContext(config());
        if (!ctx || !ctx->controller) {
            period_sec_ = 1.0 / max_rate_;
            return;
        }
        commsgs::planning_msgs::Odometry odom;
        if (!ctx->controller->GetLatestOdometry(odom)) {
            period_sec_ = 1.0 / max_rate_;
            return;
        }
        const double speed = std::hypot(odom.twist.twist.linear.x,
                                        odom.twist.twist.linear.y);
        period_sec_ = 1.0 / GetScaledRate(speed);
    }

    bool first_tick_{true};
    double min_rate_{0.1};
    double max_rate_{1.0};
    double min_speed_{0.0};
    double max_speed_{0.5};
    double d_rate_{0.9};
    double d_speed_{0.5};
    double period_sec_{1.0};
    std::chrono::steady_clock::time_point start_;
    commsgs::geometry_msgs::PoseStamped goal_;
    std::vector<commsgs::geometry_msgs::PoseStamped> goals_;
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

REGISTER_BEHAVIOR_TREE_NODE(SpeedController, "SpeedController")
