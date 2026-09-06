/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/tracking/tracking_client.hpp"
#include "autonomy/task/behavior_tree/blackboard_client.hpp"

#include <cmath>
#include <limits>
#include <memory>
#include <utility>

#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_cpp/tree_node.h"

namespace autonomy {
namespace task {
namespace tracking {
namespace {

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

TrackingClient::~TrackingClient() {
    if (node_) {
        (void)node_->DeleteReader(kShadowTargetTopic);
        (void)node_->DeleteReader(kShadowPathTopic);
    }
}

TrackingClient::Ptr TrackingClient::Create(
    navigation::NavigationClient::Ptr navigation)
{
    if (!navigation) {
        return nullptr;
    }
    return std::make_shared<TrackingClient>(std::move(navigation));
}

TrackingClient::Ptr TrackingClient::Create(
    std::shared_ptr<autolink::Node> node) {
    if (!node) {
        return nullptr;
    }
    auto client = Create(navigation::NavigationClient::Create(node));
    if (!client || !client->EnableShadowTransport(node)) {
        return nullptr;
    }
    return client;
}

bool TrackingClient::EnableShadowTransport(
    const std::shared_ptr<autolink::Node>& node) {
    if (!node) {
        return false;
    }
    if (shadow_transport_enabled_) {
        return true;
    }

    auto selection_writer = node->CreateWriter<Selection>(kShadowSelectTopic);
    if (!selection_writer) {
        return false;
    }
    SelectionPublisher selection_publisher =
        [writer = selection_writer](const Selection& value) {
            return writer && writer->Write(value);
        };

    const std::weak_ptr<TrackingClient> weak_self = shared_from_this();
    auto shadow_target_reader =
        node->CreateReader<automsgs::msgs::geometry_msgs::PoseStamped>(
            kShadowTargetTopic,
            [weak_self](const std::shared_ptr<
                        automsgs::msgs::geometry_msgs::PoseStamped>& target) {
                if (const auto self = weak_self.lock()) {
                    self->HandleShadowTarget(target);
                }
            });
    auto shadow_path_reader =
        node->CreateReader<automsgs::msgs::nav_msgs::Path>(
            kShadowPathTopic,
            [weak_self](
                const std::shared_ptr<automsgs::msgs::nav_msgs::Path>& path) {
                if (const auto self = weak_self.lock()) {
                    self->HandleShadowPath(path);
                }
            });
    if (!shadow_target_reader || !shadow_path_reader) {
        if (shadow_target_reader) {
            (void)node->DeleteReader(kShadowTargetTopic);
        }
        if (shadow_path_reader) {
            (void)node->DeleteReader(kShadowPathTopic);
        }
        return false;
    }

    node_ = node;
    selection_writer_ = std::move(selection_writer);
    selection_publisher_ = std::move(selection_publisher);
    shadow_target_reader_ = std::move(shadow_target_reader);
    shadow_path_reader_ = std::move(shadow_path_reader);
    shadow_transport_enabled_ = true;
    return true;
}

void TrackingClient::SetShared(const Ptr& client)
{
    BlackboardClientStore<TrackingClient>::SetShared(client);
    if (client) {
        navigation::NavigationClient::SetShared(client->navigation_);
    }
}

TrackingClient::Ptr TrackingClient::FromBlackboard(
    const std::shared_ptr<BT::Blackboard>& blackboard)
{
    return BlackboardClientStore<TrackingClient>::FromBlackboard(blackboard, kTrackingClientBlackboardKey);
}

TrackingClient::Ptr TrackingClient::FromNode(const BT::TreeNode& node)
{
    return BlackboardClientStore<TrackingClient>::FromNode(node, kTrackingClientBlackboardKey);
}

bool TrackingClient::ApplyGoal(
    const ::autonomy::task::proto::TrackerGoal& goal) {
    std::string target_id;
    std::optional<automsgs::msgs::geometry_msgs::PoseStamped> target_pose;

    switch (goal.target_case()) {
    case ::autonomy::task::proto::TrackerGoal::kTargetId:
        target_id = goal.target_id();
        break;
    case ::autonomy::task::proto::TrackerGoal::kTargetPose:
        target_pose = goal.target_pose();
        break;
    default:
        break;
    }

    const bool selects_person =
        goal.mode() == ::autonomy::task::proto::TRACKER_MODE_PERSON &&
        (goal.command() == ::autonomy::task::proto::TRACKER_CMD_START ||
         goal.command() == ::autonomy::task::proto::TRACKER_CMD_UPDATE_TARGET);
    if (selects_person) {
        ClearShadowState();
        Selection selection;
        selection.set_data(target_id);
        if (!selection_publisher_ || !selection_publisher_(selection)) {
            return false;
        }
    }

    mode_ = goal.mode();
    follow_distance_ = goal.follow_distance() > 0.f ? goal.follow_distance()
                                                    : kDefaultFollowDistance;
    reacquire_on_lost_ = goal.reacquire_on_lost();
    target_id_ = std::move(target_id);
    target_pose_ = std::move(target_pose);
    return true;
}

bool TrackingClient::IsTargetLocked() const
{
    if (mode_ == ::autonomy::task::proto::TRACKER_MODE_PERSON) {
        automsgs::msgs::geometry_msgs::PoseStamped target;
        automsgs::msgs::nav_msgs::Path path;
        uint64_t revision = 0;
        float distance = 0.0F;
        return GetShadowSnapshot(&target, &path, &revision, &distance);
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
    if (mode_ == ::autonomy::task::proto::TRACKER_MODE_PERSON) {
        float distance = 0.0F;
        return GetShadowDistanceToTarget(&distance) ? distance : 0.0F;
    }
    if (navigation_ && navigation_->follow_session().has_feedback()) {
        return navigation_->follow_session().latest_feedback().distance_to_goal();
    }
    return 0.f;
}

bool TrackingClient::GetShadowTarget(
    automsgs::msgs::geometry_msgs::PoseStamped* target) const {
    if (target == nullptr) {
        return false;
    }
    std::lock_guard<std::mutex> lock(shadow_mutex_);
    if (!has_shadow_target_ || !IsFresh(shadow_target_receive_time_)) {
        target->Clear();
        return false;
    }
    *target = shadow_target_;
    return true;
}

bool TrackingClient::GetShadowPath(automsgs::msgs::nav_msgs::Path* path,
                                   uint64_t* revision) const {
    if (path == nullptr || revision == nullptr) {
        return false;
    }
    automsgs::msgs::geometry_msgs::PoseStamped target;
    float distance = 0.0F;
    return GetShadowSnapshot(&target, path, revision, &distance);
}

bool TrackingClient::GetShadowDistanceToTarget(float* distance) const {
    if (distance == nullptr) {
        return false;
    }
    automsgs::msgs::geometry_msgs::PoseStamped target;
    automsgs::msgs::nav_msgs::Path path;
    uint64_t revision = 0;
    return GetShadowSnapshot(&target, &path, &revision, distance);
}

bool TrackingClient::GetShadowSnapshot(
    automsgs::msgs::geometry_msgs::PoseStamped* target,
    automsgs::msgs::nav_msgs::Path* path, uint64_t* revision,
    float* distance) const {
    if (target == nullptr || path == nullptr || revision == nullptr ||
        distance == nullptr) {
        return false;
    }

    std::lock_guard<std::mutex> lock(shadow_mutex_);
    *revision = shadow_path_revision_;
    const auto now = clock_();
    const auto is_fresh = [now](std::chrono::steady_clock::time_point time) {
        return now >= time && now - time <= kShadowDataTimeout;
    };
    if (!has_shadow_target_ || !has_shadow_path_ ||
        shadow_path_.poses().empty() ||
        !is_fresh(shadow_target_receive_time_) ||
        !is_fresh(shadow_path_receive_time_)) {
        target->Clear();
        path->Clear();
        *distance = 0.0F;
        return false;
    }

    const auto& target_position = shadow_target_.pose().position();
    const auto& robot = shadow_path_.poses(0).pose().position();
    const double value = std::hypot(target_position.x() - robot.x(),
                                    target_position.y() - robot.y());
    if (!std::isfinite(value) ||
        value > static_cast<double>(std::numeric_limits<float>::max())) {
        target->Clear();
        path->Clear();
        *distance = 0.0F;
        return false;
    }
    *target = shadow_target_;
    *path = shadow_path_;
    *distance = static_cast<float>(value);
    return true;
}

void TrackingClient::HandleShadowTarget(
    const std::shared_ptr<automsgs::msgs::geometry_msgs::PoseStamped>& target) {
    if (!target) {
        return;
    }
    std::lock_guard<std::mutex> lock(shadow_mutex_);
    shadow_target_ = *target;
    shadow_target_receive_time_ = clock_();
    has_shadow_target_ = true;
}

void TrackingClient::HandleShadowPath(
    const std::shared_ptr<automsgs::msgs::nav_msgs::Path>& path) {
    if (!path) {
        return;
    }
    std::lock_guard<std::mutex> lock(shadow_mutex_);
    shadow_path_ = *path;
    shadow_path_receive_time_ = clock_();
    ++shadow_path_revision_;
    has_shadow_path_ = true;
    if (shadow_path_.poses().empty()) {
        shadow_target_.Clear();
        has_shadow_target_ = false;
    }
}

bool TrackingClient::IsFresh(
    std::chrono::steady_clock::time_point receive_time) const {
    const auto now = clock_();
    return now >= receive_time && now - receive_time <= kShadowDataTimeout;
}

void TrackingClient::ClearShadowState() {
    std::lock_guard<std::mutex> lock(shadow_mutex_);
    shadow_target_.Clear();
    shadow_path_.Clear();
    has_shadow_target_ = false;
    has_shadow_path_ = false;
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
