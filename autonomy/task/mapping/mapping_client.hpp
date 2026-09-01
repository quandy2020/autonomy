/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <memory>
#include <optional>
#include <string>

#include "autolink/node/node.hpp"
#include <automsgs/msgs/geometry_msgs/point.pb.h>
#include <automsgs/msgs/geometry_msgs/quaternion.pb.h>
#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/transform.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist.pb.h>
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/vector3.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_with_covariance_stamped.pb.h>
#include "autonomy/task/navigation/navigation_client.hpp"
#include <automsgs/task/mapping.pb.h>

namespace BT {
class Blackboard;
class TreeNode;
}  // namespace BT

namespace autonomy {
namespace task {
namespace mapping {

constexpr char kMappingClientBlackboardKey[] = "mapping_client";

class MappingClient
{
public:
    using Ptr = std::shared_ptr<MappingClient>;

    static Ptr Create(navigation::NavigationClient::Ptr navigation);
    static Ptr Create(std::shared_ptr<autolink::Node> node);
    static void SetShared(const Ptr& client);
    static Ptr FromBlackboard(const std::shared_ptr<BT::Blackboard>& blackboard);
    static Ptr FromNode(const BT::TreeNode& node);

    void ApplyGoal(const ::autonomy::task::proto::MappingGoal& goal);

    bool LoadMap();
    bool SetInitialPose();
    bool ClearCostmap() const;

    ::autonomy::task::proto::MapCommand command() const { return command_; }
    const std::string& map_name() const { return map_name_; }
    const std::optional<automsgs::msgs::geometry_msgs::PoseWithCovarianceStamped>&
    initial_pose() const
    {
        return initial_pose_;
    }

    navigation::NavigationClient::Ptr navigation_client() const
    {
        return navigation_;
    }

    explicit MappingClient(navigation::NavigationClient::Ptr navigation);

private:
    navigation::NavigationClient::Ptr navigation_;
    ::autonomy::task::proto::MapCommand command_{
        ::autonomy::task::proto::MAP_CMD_UNSPECIFIED};
    std::string map_name_;
    std::optional<automsgs::msgs::geometry_msgs::PoseWithCovarianceStamped>
        initial_pose_;
};

}  // namespace mapping
}  // namespace task
}  // namespace autonomy
