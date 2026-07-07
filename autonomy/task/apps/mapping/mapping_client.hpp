/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <memory>
#include <optional>
#include <string>

#include "autolink/node/node.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/task/apps/navigation/navigation_client.hpp"
#include "autonomy/task/proto/mapping.pb.h"

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
    const std::optional<commsgs::geometry_msgs::PoseWithCovarianceStamped>&
    initial_pose() const
    {
        return initial_pose_;
    }

    navigation::NavigationClient::Ptr navigation_ptr() const
    {
        return navigation_;
    }

    explicit MappingClient(navigation::NavigationClient::Ptr navigation);

private:
    navigation::NavigationClient::Ptr navigation_;
    ::autonomy::task::proto::MapCommand command_{
        ::autonomy::task::proto::MAP_CMD_UNSPECIFIED};
    std::string map_name_;
    std::optional<commsgs::geometry_msgs::PoseWithCovarianceStamped>
        initial_pose_;
};

}  // namespace mapping
}  // namespace task
}  // namespace autonomy
