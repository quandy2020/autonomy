/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/charging/charging_client.hpp"
#include "autonomy/task/behavior_tree/blackboard_client.hpp"

#include "autonomy/common/logging.hpp"
#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_cpp/tree_node.h"

namespace autonomy {
namespace task {
namespace charging {
ChargingClient::ChargingClient(navigation::NavigationClient::Ptr navigation)
    : navigation_(std::move(navigation))
{
    dock_pose_.mutable_header()->set_frame_id("map");
    dock_pose_.mutable_pose()->mutable_orientation()->set_w(1.0);
}

ChargingClient::Ptr ChargingClient::Create(
    navigation::NavigationClient::Ptr navigation)
{
    if (!navigation) {
        return nullptr;
    }
    return std::make_shared<ChargingClient>(std::move(navigation));
}

ChargingClient::Ptr ChargingClient::Create(std::shared_ptr<autolink::Node> node)
{
    return Create(navigation::NavigationClient::Create(std::move(node)));
}

void ChargingClient::SetShared(const Ptr& client)
{
    BlackboardClientStore<ChargingClient>::SetShared(client);
    if (client) {
        navigation::NavigationClient::SetShared(client->navigation_);
    }
}

ChargingClient::Ptr ChargingClient::FromBlackboard(
    const std::shared_ptr<BT::Blackboard>& blackboard)
{
    return BlackboardClientStore<ChargingClient>::FromBlackboard(blackboard, kChargingClientBlackboardKey);
}

ChargingClient::Ptr ChargingClient::FromNode(const BT::TreeNode& node)
{
    return BlackboardClientStore<ChargingClient>::FromNode(node, kChargingClientBlackboardKey);
}

void ChargingClient::ApplyGoal(const ::autonomy::task::proto::ChargingGoal& goal)
{
    max_search_radius_ = goal.max_search_radius() > 0.f ? goal.max_search_radius()
                                                        : 3.f;
    search_complete_ = false;
    charger_visible_ = false;
    dock_connected_ = false;

    dock_station_id_.clear();
    switch (goal.dock_target_case()) {
    case ::autonomy::task::proto::ChargingGoal::kDockStationId:
        dock_station_id_ = goal.dock_station_id();
        break;
    case ::autonomy::task::proto::ChargingGoal::kDockPose:
        dock_pose_ = goal.dock_pose();
        break;
    default:
        break;
    }
}

bool ChargingClient::RunDockSearch()
{
    search_complete_ = true;
    AINFO << "ChargingClient: dock search complete (radius="
          << max_search_radius_ << " m)";
    return true;
}

bool ChargingClient::IsChargerVisible() const
{
    return search_complete_ && (charger_visible_ || !dock_station_id_.empty() ||
                                dock_pose_.pose().position().x() != 0.0 ||
                                dock_pose_.pose().position().y() != 0.0);
}

bool ChargingClient::MarkConnected()
{
    charger_visible_ = true;
    dock_connected_ = true;
    AINFO << "ChargingClient: dock connected";
    return true;
}

void ChargingClient::CancelActiveMotion()
{
    if (navigation_) {
        navigation_->CancelActiveMotion();
    }
}

}  // namespace charging
}  // namespace task
}  // namespace autonomy
