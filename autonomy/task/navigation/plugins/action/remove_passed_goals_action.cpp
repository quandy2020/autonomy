/*
 * Copyright 2026 The Openbot Authors
 *
 * Drop leading waypoints the robot has already reached so multi-goal replans
 * only consider the remaining route.
 */

#include <cmath>
#include <string>
#include <vector>

#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>

#include "autonomy/common/logging.hpp"
#include "autonomy/task/behavior_tree/plugins/bt_node_base.hpp"
#include "autonomy/transform/buffer.hpp"

namespace autonomy::task::plugins::navigation {
namespace {

bool LookupRobotXy(const std::string& global_frame,
                   const std::string& robot_frame, double* x, double* y)
{
    if (x == nullptr || y == nullptr) {
        return false;
    }
    auto* buffer = ::autonomy::transform::Buffer::Instance();
    if (buffer == nullptr) {
        return false;
    }
    ::automsgs::msgs::builtin_interfaces::Time time;
    time.set_sec(0);
    time.set_nanosec(0);
    try {
        const auto tf =
            buffer->lookupTransform(global_frame, robot_frame, time, 0.1f);
        *x = tf.transform().translation().x();
        *y = tf.transform().translation().y();
        return true;
    } catch (const std::exception& ex) {
        AWARN_EVERY(20) << "RemovePassedGoals: TF lookup failed: " << ex.what();
        return false;
    }
}

}  // namespace

class RemovePassedGoalsAction : public BtSyncAction
{
public:
    RemovePassedGoalsAction(const std::string& name,
                            const BT::NodeConfig& config)
        : BtSyncAction(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<
                std::vector<automsgs::msgs::geometry_msgs::PoseStamped>>(
                "input_goals"),
            BT::OutputPort<
                std::vector<automsgs::msgs::geometry_msgs::PoseStamped>>(
                "output_goals"),
            BT::InputPort<double>("radius", 0.6, "m"),
            BT::InputPort<std::string>("global_frame", "map", ""),
            BT::InputPort<std::string>("robot_base_frame", "base_link", ""),
        };
    }

protected:
    BT::NodeStatus OnExecute() override
    {
        std::vector<automsgs::msgs::geometry_msgs::PoseStamped> goals;
        if (!getInput("input_goals", goals)) {
            return BT::NodeStatus::FAILURE;
        }

        double radius = 0.6;
        std::string global_frame = "map";
        std::string robot_frame = "base_link";
        getInput("radius", radius);
        getInput("global_frame", global_frame);
        getInput("robot_base_frame", robot_frame);

        double rx = 0.0;
        double ry = 0.0;
        if (!LookupRobotXy(global_frame, robot_frame, &rx, &ry)) {
            setOutput("output_goals", goals);
            return BT::NodeStatus::SUCCESS;
        }

        const double r2 = radius * radius;
        size_t removed = 0;
        while (!goals.empty()) {
            const auto& p = goals.front().pose().position();
            const double dx = p.x() - rx;
            const double dy = p.y() - ry;
            if ((dx * dx + dy * dy) > r2) {
                break;
            }
            goals.erase(goals.begin());
            ++removed;
        }

        if (removed > 0) {
            AINFO_EVERY(5) << "RemovePassedGoals: removed " << removed
                           << " passed waypoint(s); remaining=" << goals.size();
        }

        setOutput("output_goals", goals);
        if (config().blackboard) {
            config().blackboard->set("goals", goals);
            config().blackboard->set("number_of_goals",
                                     static_cast<int>(goals.size()));
            if (!goals.empty()) {
                config().blackboard->set("goal", goals.back());
            }
        }
        return BT::NodeStatus::SUCCESS;
    }
};

}  // namespace autonomy::task::plugins::navigation

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<
        autonomy::task::plugins::navigation::RemovePassedGoalsAction>(
        "RemovePassedGoals");
}
