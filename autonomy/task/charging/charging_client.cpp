/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/charging/charging_client.hpp"

#include "autonomy/common/logging.hpp"
#include "autonomy/task/behavior_tree/blackboard_client.hpp"
#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_cpp/tree_node.h"

namespace autonomy {
namespace task {
namespace charging {

ChargingClient::ChargingClient(navigation::NavigationClient::Ptr navigation,
                               std::shared_ptr<autolink::Node> node)
    : navigation_(std::move(navigation)), node_(std::move(node)) {
    dock_pose_.mutable_header()->set_frame_id("map");
    dock_pose_.mutable_pose()->mutable_orientation()->set_w(1.0);
}

ChargingClient::Ptr ChargingClient::Create(
    navigation::NavigationClient::Ptr navigation,
    std::shared_ptr<autolink::Node> node) {
    if (!navigation) {
        return nullptr;
    }
    return std::make_shared<ChargingClient>(std::move(navigation), std::move(node));
}

ChargingClient::Ptr ChargingClient::Create(std::shared_ptr<autolink::Node> node) {
    return Create(navigation::NavigationClient::Create(node), node);
}

void ChargingClient::SetShared(const Ptr& client) {
    BlackboardClientStore<ChargingClient>::SetShared(client);
    if (client) {
        navigation::NavigationClient::SetShared(client->navigation_);
    }
}

ChargingClient::Ptr ChargingClient::FromBlackboard(
    const std::shared_ptr<BT::Blackboard>& blackboard) {
    return BlackboardClientStore<ChargingClient>::FromBlackboard(
        blackboard, kChargingClientBlackboardKey);
}

ChargingClient::Ptr ChargingClient::FromNode(const BT::TreeNode& node) {
    return BlackboardClientStore<ChargingClient>::FromNode(
        node, kChargingClientBlackboardKey);
}

bool ChargingClient::Configure(
    const ::autonomy::task::proto::DockingOptions& options) {
    auto_recharge_ = options.auto_recharge();
    max_hold_sec_ = auto_recharge_.max_hold_charge_sec() > 0.f
                        ? auto_recharge_.max_hold_charge_sec()
                        : 14400.0;
    if (!engine_.Configure(node_, navigation_, options)) {
        return false;
    }
    engine_.Activate();
    return true;
}

void ChargingClient::ApplyGoal(const ::autonomy::task::proto::ChargingGoal& goal) {
    max_search_radius_ =
        goal.max_search_radius() > 0.f ? goal.max_search_radius() : 3.f;
    max_retry_count_ = goal.max_retry_count();
    max_staging_time_ =
        goal.max_staging_time() > 0.f ? goal.max_staging_time() : 1000.f;
    max_undocking_time_ =
        goal.max_undocking_time() > 0.f ? goal.max_undocking_time() : 30.f;
    navigate_to_staging_ = goal.has_navigate_to_staging_pose()
                               ? goal.navigate_to_staging_pose()
                               : true;
    dock_type_ = goal.dock_type();
    search_complete_ = false;
    charger_visible_ = false;
    dock_connected_ = false;
    dock_station_id_.clear();
    use_dock_id_ = goal.use_dock_id();
    if (goal.has_battery_target_percent()) {
        battery_target_percent_ = goal.battery_target_percent();
    } else if (auto_recharge_.enable()) {
        battery_target_percent_ = auto_recharge_.resume_percent() > 0.f
                                      ? auto_recharge_.resume_percent()
                                      : 95.f;
    } else {
        battery_target_percent_ = 0.f;
    }
    switch (goal.dock_target_case()) {
    case ::autonomy::task::proto::ChargingGoal::kDockStationId:
        dock_station_id_ = goal.dock_station_id();
        use_dock_id_ = true;
        break;
    case ::autonomy::task::proto::ChargingGoal::kDockPose:
        dock_pose_ = goal.dock_pose();
        use_dock_id_ = false;
        break;
    default:
        if (goal.use_dock_id() && !goal.dock_station_id().empty()) {
            dock_station_id_ = goal.dock_station_id();
            use_dock_id_ = true;
        }
        break;
    }
    if (use_dock_id_ && dock_station_id_.empty()) {
        dock_station_id_ = engine_.SelectDockId(auto_recharge_.preferred_dock_id());
        use_dock_id_ = !dock_station_id_.empty();
        AINFO << "charging default dock_id=" << dock_station_id_;
    }
}

DockRequest ChargingClient::MakeDockRequest() const {
    DockRequest req;
    req.use_dock_id = use_dock_id_ && !dock_station_id_.empty();
    req.dock_id = dock_station_id_;
    req.dock_type = dock_type_;
    req.dock_pose = dock_pose_;
    req.navigate_to_staging = navigate_to_staging_;
    req.max_staging_time_sec = max_staging_time_;
    req.max_retries = max_retry_count_;
    req.hold_until_percent = battery_target_percent_;
    req.max_hold_sec = max_hold_sec_;
    return req;
}

UndockRequest ChargingClient::MakeUndockRequest() const {
    UndockRequest req;
    req.dock_type = dock_type_;
    req.max_undocking_time_sec = max_undocking_time_;
    return req;
}

bool ChargingClient::StartDock() {
    return engine_.StartDock(MakeDockRequest());
}

bool ChargingClient::StartUndock() {
    return engine_.StartUndock(MakeUndockRequest());
}

DockingEngine::RunState ChargingClient::Tick() { return engine_.Tick(); }

void ChargingClient::CancelActiveMotion() {
    engine_.Cancel();
    if (navigation_) {
        navigation_->CancelActiveMotion();
    }
}

bool ChargingClient::RunDockSearch() {
    search_complete_ = true;
    charger_visible_ = use_dock_id_ ? !dock_station_id_.empty()
                                    : (dock_pose_.pose().position().x() != 0.0 ||
                                       dock_pose_.pose().position().y() != 0.0);
    AINFO << "dock search complete visible=" << charger_visible_;
    return charger_visible_ || use_dock_id_;
}

bool ChargingClient::IsChargerVisible() const {
    if (engine_.phase() == ::autonomy::task::proto::DOCK_PHASE_CONTROLLING ||
        engine_.phase() == ::autonomy::task::proto::DOCK_PHASE_WAIT_FOR_CHARGE) {
        return true;
    }
    return search_complete_ &&
           (charger_visible_ || !dock_station_id_.empty() ||
            dock_pose_.pose().position().x() != 0.0 ||
            dock_pose_.pose().position().y() != 0.0);
}

bool ChargingClient::MarkConnected() {
    charger_visible_ = true;
    dock_connected_ = true;
    return true;
}

}  // namespace charging
}  // namespace task
}  // namespace autonomy
