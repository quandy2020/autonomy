/*
 * Copyright 2026 The Openbot Authors
 *
 * Exploration BT facade: ExplorerInterface + navigation RPC.
 */

#pragma once

#include <memory>
#include <string>

#include "autolink/node/node.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/map_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/commsgs/sensor_msgs.hpp"
#include "autonomy/exploration/common/explorer_interface.hpp"
#include "autonomy/exploration/exploration_options.hpp"
#include "autonomy/exploration/proto/exploration_options.pb.h"
#include "autonomy/task/apps/navigation/navigation_client.hpp"

namespace BT {
class Blackboard;
class TreeNode;
}  // namespace BT

namespace autonomy {
namespace task {
namespace exploration {

constexpr char kExplorationClientBlackboardKey[] = "exploration_client";

/** Task-layer facade over RGBD ExplorerInterface; motion via NavigationClient. */
class ExplorationClient
{
public:
    using Ptr = std::shared_ptr<ExplorationClient>;

    static Ptr Create(navigation::NavigationClient::Ptr navigation);
    static Ptr Create(std::shared_ptr<autolink::Node> node);
    static void SetShared(const Ptr& client);
    static Ptr FromBlackboard(const std::shared_ptr<BT::Blackboard>& blackboard);
    static Ptr FromNode(const BT::TreeNode& node);

    navigation::NavigationClient::Ptr navigation_ptr() const
    {
        return navigation_;
    }

    navigation::NavigationClient& navigation() { return *navigation_; }

    void SetOptions(
        const ::autonomy::exploration::proto::ExplorationOptions& options);
    void SetMapName(const std::string& map_name);
    const std::string& map_name() const { return map_name_; }

    void SetExplorationArea(const commsgs::geometry_msgs::Polygon& area);
    void UseDefaultExplorationArea();

    void UpdateOdometry(const commsgs::planning_msgs::Odometry& odom);
    void UpdateDepth(const commsgs::sensor_msgs::Image& depth,
                     const commsgs::sensor_msgs::CameraInfo& info,
                     const commsgs::geometry_msgs::Transform& map_t_camera);

    bool HasFrontier() const;
    bool SelectNextFrontier(commsgs::geometry_msgs::PoseStamped& goal);
    void MarkFrontierVisited();

    float exploration_progress() const;
    float explored_area_m2() const;
    bool IsExplorationFinished() const;

    commsgs::map_msgs::OccupancyGrid GetOccupancyGrid(
        const std::string& frame_id = "map") const;
    bool SaveExplorationMap(const std::string& map_name) const;

    void CancelActiveMotion();

    explicit ExplorationClient(navigation::NavigationClient::Ptr navigation);

private:
    void EnsureExplorer();

    navigation::NavigationClient::Ptr navigation_;
    ::autonomy::exploration::common::ExplorerInterface::SharedPtr explorer_;
    std::string map_name_{"exploration_map"};
    std::string map_frame_{"map"};
};

}  // namespace exploration
}  // namespace task
}  // namespace autonomy
