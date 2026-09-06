/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <automsgs/msgs/geometry_msgs/point.pb.h>
#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/quaternion.pb.h>
#include <automsgs/msgs/geometry_msgs/transform.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist.pb.h>
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/vector3.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/std_msgs/string.pb.h>
#include <automsgs/task/tracker.pb.h>
#include "autolink/node/node.hpp"
#include "autonomy/task/navigation/navigation_client.hpp"
#include "behaviortree_cpp/basic_types.h"

namespace BT {
class Blackboard;
class TreeNode;
}  // namespace BT

namespace autonomy {
namespace task {
namespace tracking {

constexpr char kTrackingClientBlackboardKey[] = "tracking_client";
constexpr double kDefaultFollowDistance = 1.5;
constexpr char kShadowSelectTopic[] = "/perception/shadow/select";
constexpr char kShadowTargetTopic[] = "/perception/shadow/target";
constexpr char kShadowPathTopic[] = "/perception/shadow/path";

class TrackingClientTestApi;

class TrackingClient : public std::enable_shared_from_this<TrackingClient>
{
public:
    using Ptr = std::shared_ptr<TrackingClient>;
    using Clock = std::function<std::chrono::steady_clock::time_point()>;
    using Selection = automsgs::msgs::std_msgs::String;
    using SelectionPublisher = std::function<bool(const Selection&)>;

    inline static constexpr std::chrono::milliseconds kShadowDataTimeout{500};

    static Ptr Create(navigation::NavigationClient::Ptr navigation);
    static Ptr Create(std::shared_ptr<autolink::Node> node);
    static void SetShared(const Ptr& client);
    static Ptr FromBlackboard(const std::shared_ptr<BT::Blackboard>& blackboard);
    static Ptr FromNode(const BT::TreeNode& node);

    bool EnableShadowTransport(const std::shared_ptr<autolink::Node>& node);
    bool ApplyGoal(const ::autonomy::task::proto::TrackerGoal& goal);

    bool IsTargetLocked() const;
    bool ComputeFollowGoal(automsgs::msgs::geometry_msgs::PoseStamped& goal) const;
    float DistanceToTarget() const;
    bool GetShadowTarget(
        automsgs::msgs::geometry_msgs::PoseStamped* target) const;
    bool GetShadowPath(automsgs::msgs::nav_msgs::Path* path,
                       uint64_t* revision) const;
    bool GetShadowDistanceToTarget(float* distance) const;
    bool GetShadowSnapshot(automsgs::msgs::geometry_msgs::PoseStamped* target,
                           automsgs::msgs::nav_msgs::Path* path,
                           uint64_t* revision, float* distance) const;

    const std::string& target_id() const { return target_id_; }
    double follow_distance() const { return follow_distance_; }
    bool reacquire_on_lost() const {
        return reacquire_on_lost_;
    }
    bool shadow_transport_enabled() const {
        return shadow_transport_enabled_;
    }
    ::autonomy::task::proto::TrackerMode mode() const { return mode_; }

    navigation::NavigationClient::Ptr navigation_client() const
    {
        return navigation_;
    }

    void CancelActiveMotion();

    explicit TrackingClient(navigation::NavigationClient::Ptr navigation);
    ~TrackingClient();

private:
    friend class TrackingClientTestApi;

    void HandleShadowTarget(
        const std::shared_ptr<automsgs::msgs::geometry_msgs::PoseStamped>&
            target);
    void HandleShadowPath(
        const std::shared_ptr<automsgs::msgs::nav_msgs::Path>& path);
    bool IsFresh(std::chrono::steady_clock::time_point receive_time) const;
    void ClearShadowState();

    navigation::NavigationClient::Ptr navigation_;
    ::autonomy::task::proto::TrackerMode mode_{
        ::autonomy::task::proto::TRACKER_MODE_UNSPECIFIED};
    std::string target_id_;
    std::optional<automsgs::msgs::geometry_msgs::PoseStamped> target_pose_;
    double follow_distance_{kDefaultFollowDistance};
    bool reacquire_on_lost_{true};

    std::shared_ptr<autolink::Node> node_;
    std::shared_ptr<autolink::Writer<Selection>> selection_writer_;
    std::shared_ptr<
        autolink::Reader<automsgs::msgs::geometry_msgs::PoseStamped>>
        shadow_target_reader_;
    std::shared_ptr<autolink::Reader<automsgs::msgs::nav_msgs::Path>>
        shadow_path_reader_;
    SelectionPublisher selection_publisher_;
    Clock clock_{[]() { return std::chrono::steady_clock::now(); }};
    bool shadow_transport_enabled_{false};

    mutable std::mutex shadow_mutex_;
    automsgs::msgs::geometry_msgs::PoseStamped shadow_target_;
    automsgs::msgs::nav_msgs::Path shadow_path_;
    std::chrono::steady_clock::time_point shadow_target_receive_time_{};
    std::chrono::steady_clock::time_point shadow_path_receive_time_{};
    uint64_t shadow_path_revision_{0};
    bool has_shadow_target_{false};
    bool has_shadow_path_{false};
};

namespace internal {

/**
 * Stateful, middleware-free seam used by FollowShadowPath's BT wrapper.
 * Production operations delegate to the existing FollowPath ActionSession.
 */
class ShadowPathExecutor
{
public:
    using Path = automsgs::msgs::nav_msgs::Path;

    struct Operations {
        std::function<bool(Path*, uint64_t*)> get_path;
        std::function<bool()> action_ready;
        std::function<void(const Path&, const std::string&)> begin;
        std::function<BT::NodeStatus(int*, std::string*)> tick;
        std::function<void()> cancel;
    };

    explicit ShadowPathExecutor(Operations operations);

    BT::NodeStatus Tick(const std::string& controller_id, int* error_code,
                        std::string* error_message);
    void Halt();
    bool active() const {
        return active_;
    }

private:
    void Fail(int code, const std::string& message, int* error_code,
              std::string* error_message);

    Operations operations_;
    uint64_t active_revision_{0};
    uint64_t completed_revision_{0};
    bool has_active_revision_{false};
    bool has_completed_revision_{false};
    bool active_{false};
};

}  // namespace internal

}  // namespace tracking
}  // namespace task
}  // namespace autonomy
