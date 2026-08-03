/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/apps/mapping/mapping_client.hpp"

#include "autonomy/common/logging.hpp"
#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_cpp/tree_node.h"

namespace autonomy {
namespace task {
namespace mapping {
namespace {

std::weak_ptr<MappingClient> g_shared_client;

}  // namespace

MappingClient::MappingClient(navigation::NavigationClient::Ptr navigation)
    : navigation_(std::move(navigation))
{
}

MappingClient::Ptr MappingClient::Create(
    navigation::NavigationClient::Ptr navigation)
{
    if (!navigation) {
        return nullptr;
    }
    return std::make_shared<MappingClient>(std::move(navigation));
}

MappingClient::Ptr MappingClient::Create(std::shared_ptr<autolink::Node> node)
{
    return Create(navigation::NavigationClient::Create(std::move(node)));
}

void MappingClient::SetShared(const Ptr& client)
{
    g_shared_client = client;
    if (client) {
        navigation::NavigationClient::SetShared(client->navigation_);
    }
}

MappingClient::Ptr MappingClient::FromBlackboard(
    const std::shared_ptr<BT::Blackboard>& blackboard)
{
    if (blackboard) {
        Ptr client;
        if (blackboard->get(kMappingClientBlackboardKey, client) && client) {
            return client;
        }
    }
    return g_shared_client.lock();
}

MappingClient::Ptr MappingClient::FromNode(const BT::TreeNode& node)
{
    return FromBlackboard(node.config().blackboard);
}

void MappingClient::ApplyGoal(const ::autonomy::task::proto::MappingGoal& goal)
{
    command_ = goal.command();
    map_name_.clear();
    initial_pose_.reset();

    switch (goal.params_case()) {
    case ::autonomy::task::proto::MappingGoal::kMapName:
        map_name_ = goal.map_name();
        break;
    case ::autonomy::task::proto::MappingGoal::kInitialPose:
        initial_pose_ =
            goal.initial_pose();
        break;
    default:
        break;
    }
}

bool MappingClient::LoadMap()
{
    AINFO << "MappingClient: load map '" << map_name_ << "'";
    return !map_name_.empty();
}

bool MappingClient::SetInitialPose()
{
    if (!initial_pose_) {
        return false;
    }
    AINFO << "MappingClient: set initial pose";
    return true;
}

bool MappingClient::ClearCostmap() const
{
    if (!navigation_) {
        return false;
    }
    return navigation_->ClearCostmap();
}

}  // namespace mapping
}  // namespace task
}  // namespace autonomy
