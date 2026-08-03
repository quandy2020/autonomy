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
#include "autonomy/task/proto/charging.pb.h"

namespace BT {
class Blackboard;
class TreeNode;
}  // namespace BT

namespace autonomy {
namespace task {
namespace charging {

constexpr char kChargingClientBlackboardKey[] = "charging_client";

class ChargingClient
{
public:
    using Ptr = std::shared_ptr<ChargingClient>;

    static Ptr Create(navigation::NavigationClient::Ptr navigation);
    static Ptr Create(std::shared_ptr<autolink::Node> node);
    static void SetShared(const Ptr& client);
    static Ptr FromBlackboard(const std::shared_ptr<BT::Blackboard>& blackboard);
    static Ptr FromNode(const BT::TreeNode& node);

    void ApplyGoal(const ::autonomy::task::proto::ChargingGoal& goal);

    bool RunDockSearch();
    bool IsChargerVisible() const;
    bool MarkConnected();

    const std::string& dock_station_id() const { return dock_station_id_; }
    const automsgs::msgs::geometry_msgs::PoseStamped& dock_pose() const
    {
        return dock_pose_;
    }
    float battery_target_percent() const { return battery_target_percent_; }

    navigation::NavigationClient::Ptr navigation_ptr() const
    {
        return navigation_;
    }

    void CancelActiveMotion();

    explicit ChargingClient(navigation::NavigationClient::Ptr navigation);

private:
    navigation::NavigationClient::Ptr navigation_;
    std::string dock_station_id_;
    automsgs::msgs::geometry_msgs::PoseStamped dock_pose_;
    double max_search_radius_{3.0};
    float battery_target_percent_{100.f};
    bool search_complete_{false};
    bool charger_visible_{false};
    bool dock_connected_{false};
};

}  // namespace charging
}  // namespace task
}  // namespace autonomy
