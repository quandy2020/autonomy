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
#include "autonomy/task/apps/navigation/navigation_client.hpp"
#include "autonomy/task/proto/tracker.pb.h"

namespace BT {
class Blackboard;
class TreeNode;
}  // namespace BT

namespace autonomy {
namespace task {
namespace tracking {

constexpr char kTrackingClientBlackboardKey[] = "tracking_client";
constexpr double kDefaultFollowDistance = 1.5;

class TrackingClient
{
public:
    using Ptr = std::shared_ptr<TrackingClient>;

    static Ptr Create(navigation::NavigationClient::Ptr navigation);
    static Ptr Create(std::shared_ptr<autolink::Node> node);
    static void SetShared(const Ptr& client);
    static Ptr FromBlackboard(const std::shared_ptr<BT::Blackboard>& blackboard);
    static Ptr FromNode(const BT::TreeNode& node);

    void ApplyGoal(const ::autonomy::task::proto::TrackerGoal& goal);

    bool IsTargetLocked() const;
    bool ComputeFollowGoal(automsgs::msgs::geometry_msgs::PoseStamped& goal) const;
    float DistanceToTarget() const;

    const std::string& target_id() const { return target_id_; }
    double follow_distance() const { return follow_distance_; }
    ::autonomy::task::proto::TrackerMode mode() const { return mode_; }

    navigation::NavigationClient::Ptr navigation_ptr() const
    {
        return navigation_;
    }

    void CancelActiveMotion();

    explicit TrackingClient(navigation::NavigationClient::Ptr navigation);

private:
    navigation::NavigationClient::Ptr navigation_;
    ::autonomy::task::proto::TrackerMode mode_{
        ::autonomy::task::proto::TRACKER_MODE_UNSPECIFIED};
    std::string target_id_;
    std::optional<automsgs::msgs::geometry_msgs::PoseStamped> target_pose_;
    double follow_distance_{kDefaultFollowDistance};
    bool reacquire_on_lost_{true};
};

}  // namespace tracking
}  // namespace task
}  // namespace autonomy
