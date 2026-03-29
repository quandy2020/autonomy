/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <string>

#include "autolink/autolink.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/transform/buffer.hpp"
#include "behaviortree_cpp/action_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

/**
 * @brief A BT::ActionNodeBase to shorten path to some distance around robot
 */
class TruncatePathLocal : public BT::ActionNodeBase
{
public:
    /**
     * @brief A nav2_behavior_tree::TruncatePathLocal constructor
     * @param xml_tag_name Name for the XML tag for this node
     * @param conf BT node configuration
     */
    TruncatePathLocal(const std::string& xml_tag_name,
                      const BT::NodeConfiguration& conf);

    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing basic ports along with node-specific
     * ports
     */
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<commsgs::planning_msgs::Path>("input_path",
                                                        "Original Path"),
            BT::OutputPort<commsgs::planning_msgs::Path>(
                "output_path",
                "Path truncated to a certain distance around robot"),
            BT::InputPort<double>("distance_forward", 8.0,
                                  "Distance in forward direction"),
            BT::InputPort<double>("distance_backward", 4.0,
                                  "Distance in backward direction"),
            BT::InputPort<std::string>("robot_frame", "base_link",
                                       "Robot base frame id"),
            BT::InputPort<double>("transform_tolerance", 0.2,
                                  "Transform lookup tolerance"),
            BT::InputPort<commsgs::geometry_msgs::PoseStamped>(
                "pose",
                "Manually specified pose to be used if overriding current "
                "robot pose"),
            BT::InputPort<double>("angular_distance_weight", 0.0,
                                  "Weight of angular distance relative to "
                                  "positional distance when finding which path "
                                  "pose is closest to robot. Not applicable on "
                                  "paths without orientations assigned"),
            BT::InputPort<double>(
                "max_robot_pose_search_dist",
                std::numeric_limits<double>::infinity(),
                "Maximum forward integrated distance along the path (starting "
                "from the last detected pose) "
                "to bound the search for the closest pose to the robot. When "
                "set to infinity (default), "
                "whole path is searched every time"),
        };
    }

private:
    /**
     * @brief The other (optional) override required by a BT action.
     */
    void halt() override {}

    /**
     * @brief The main override required by a BT action
     * @return BT::NodeStatus Status of tick execution
     */
    BT::NodeStatus tick() override;

    /**
     * @brief Get either specified input pose or robot pose in path frame
     * @param path_frame_id Frame ID of path
     * @param pose Output pose
     * @return True if succeeded
     */
    bool getRobotPose(std::string path_frame_id,
                      commsgs::geometry_msgs::PoseStamped& pose);

    /**
     * @brief A custom pose distance method which takes angular distance into
     * account in addition to spatial distance (to improve picking a correct
     * pose near cusps and loops)
     * @param pose1 Distance is computed between this pose and pose2
     * @param pose2 Distance is computed between this pose and pose1
     * @param angular_distance_weight Weight of angular distance relative to
     * spatial distance (1.0 means that 1 radian of angular distance corresponds
     * to 1 meter of spatial distance)
     */
    static double poseDistance(const commsgs::geometry_msgs::PoseStamped& pose1,
                               const commsgs::geometry_msgs::PoseStamped& pose2,
                               const double angular_distance_weight);

    std::shared_ptr<autonomy::transform::Buffer> tf_buffer_;
    std::shared_ptr<::autolink::Node> node_;

    commsgs::planning_msgs::Path path_;
    std::vector<commsgs::geometry_msgs::PoseStamped>::iterator
        closest_pose_detection_begin_;
};

}  // namespace action
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
