/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/apps/tracking/tracking_client.hpp"

#include <cmath>

#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_cpp/tree_node.h"

namespace autonomy {
namespace task {
namespace tracking {
namespace {

std::weak_ptr<TrackingClient> g_shared_client;

double YawFromPose(const automsgs::msgs::geometry_msgs::Pose& pose)
{
    const auto& q = pose.orientation();
    return std::atan2(2.0 * (q.w()* q.z() + q.x()* q.y()),
                      1.0 - 2.0 * (q.y()* q.y() + q.z()* q.z()));
}

}  // namespace

TrackingClient::TrackingClient(navigation::NavigationClient::Ptr navigation)
    : navigation_(std::move(navigation))
{
}

TrackingClient::Ptr TrackingClient::Create(
    navigation::NavigationClient::Ptr navigation)
{
    if (!navigation) {
        return nullptr;
    }
    return std::make_shared<TrackingClient>(std::move(navigation));
}

TrackingClient::Ptr TrackingClient::Create(std::shared_ptr<autolink::Node> node)
{
    return Create(navigation::NavigationClient::Create(std::move(node)));
}

void TrackingClient::SetShared(const Ptr& client)
{
    g_shared_client = client;
    if (client) {
        navigation::NavigationClient::SetShared(client->navigation_);
    }
}

TrackingClient::Ptr TrackingClient::FromBlackboard(
    const std::shared_ptr<BT::Blackboard>& blackboard)
{
    if (blackboard) {
        Ptr client;
        if (blackboard->get(kTrackingClientBlackboardKey, client) && client) {
            return client;
        }
    }
    return g_shared_client.lock();
}

TrackingClient::Ptr TrackingClient::FromNode(const BT::TreeNode& node)
{
    return FromBlackboard(node.config().blackboard);
}

void TrackingClient::ApplyGoal(const ::autonomy::task::proto::TrackerGoal& goal)
{
    mode_ = goal.mode();
    follow_distance_ = goal.follow_distance() > 0.f ? goal.follow_distance()
                                                   : kDefaultFollowDistance;
    reacquire_on_lost_ = goal.reacquire_on_lost();

    target_id_.clear();
    target_pose_.reset();

    switch (goal.target_case()) {
    case ::autonomy::task::proto::TrackerGoal::kTargetId:
        target_id_ = goal.target_id();
        break;
    case ::autonomy::task::proto::TrackerGoal::kTargetPose:
        target_pose_ =
            goal.target_pose();
        break;
    default:
        break;
    }
}

bool TrackingClient::IsTargetLocked() const
{
    if (mode_ == ::autonomy::task::proto::TRACKER_MODE_PERSON) {
        return !target_id_.empty() || target_pose_.has_value();
    }
    if (!target_id_.empty()) {
        return true;
    }
    return target_pose_.has_value();
}

bool TrackingClient::ComputeFollowGoal(
    automsgs::msgs::geometry_msgs::PoseStamped& goal) const
{
    if (!target_pose_.has_value()) {
        if (target_id_.empty()) {
            return false;
        }
        goal.mutable_header()->set_frame_id("map");
        goal.mutable_pose()->mutable_position()->set_x(0.0);
        goal.mutable_pose()->mutable_position()->set_y(0.0);
        goal.mutable_pose()->mutable_orientation()->set_w(1.0);
        return true;
    }

    goal = *target_pose_;
    const double yaw = YawFromPose(goal.pose());
    goal.mutable_pose()->mutable_position()->set_x(
        goal.pose().position().x() - follow_distance_ * std::cos(yaw));
    goal.mutable_pose()->mutable_position()->set_y(
        goal.pose().position().y() - follow_distance_ * std::sin(yaw));
    return true;
}

float TrackingClient::DistanceToTarget() const
{
    if (navigation_ && navigation_->follow_session().has_feedback()) {
        return navigation_->follow_session().latest_feedback().distance_to_goal();
    }
    return 0.f;
}

void TrackingClient::CancelActiveMotion()
{
    if (navigation_) {
        navigation_->CancelActiveMotion();
    }
}

}  // namespace tracking
}  // namespace task
}  // namespace autonomy
