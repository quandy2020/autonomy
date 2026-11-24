/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file remove_passed_goals_action.cpp
 * @brief Remove passed goals action.
 */

#include <cmath>
#include <string>
#include <vector>

#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>

#include "autonomy/common/logging.hpp"
#include "autonomy/common/nav_log.hpp"
#include "autonomy/common/passed_waypoint.hpp"
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
            BT::InputPort<double>("along_track", 0.35,
                                  "m past a via toward the next via"),
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
        double along_track = 0.35;
        std::string global_frame = "map";
        std::string robot_frame = "base_link";
        getInput("radius", radius);
        getInput("along_track", along_track);
        getInput("global_frame", global_frame);
        getInput("robot_base_frame", robot_frame);

        double rx = 0.0;
        double ry = 0.0;
        if (!LookupRobotXy(global_frame, robot_frame, &rx, &ry)) {
            setOutput("output_goals", goals);
            return BT::NodeStatus::SUCCESS;
        }

        size_t removed = 0;
        // Keep the final pose until GoalReached. Prefix vias drop when the
        // robot is inside radius OR has already progressed toward the next via
        // (so a 0.25 Hz replan still skips points the robot drove past).
        while (goals.size() > 1) {
            const bool near = ::autonomy::common::WaypointWithinRadius(
                goals.front(), rx, ry, radius);
            const bool passed = ::autonomy::common::WaypointPassedToward(
                goals.front(), goals[1], rx, ry, along_track);
            if (!near && !passed) {
                break;
            }
            goals.erase(goals.begin());
            ++removed;
        }

        if (removed > 0) {
            AINFO << "nav prune passed n=" << removed
                  << " remain=" << goals.size() << " xy="
                  << ::autonomy::common::NavXY{rx, ry};
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
