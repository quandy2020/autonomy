/*
 * Copyright 2026 The Openbot Authors
 *
 * Exploration BT facade: frontier queue + navigation RPC delegation.
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "autolink/node/node.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/task/apps/navigation/navigation_client.hpp"

namespace BT {
class Blackboard;
class TreeNode;
}  // namespace BT

namespace autonomy {
namespace task {
namespace exploration {

constexpr char kExplorationClientBlackboardKey[] = "exploration_client";

/** Frontier queue and map metadata; motion uses NavigationClient RPC. */
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

    void SetMapName(const std::string& map_name);
    const std::string& map_name() const { return map_name_; }

    void SetExplorationArea(const commsgs::geometry_msgs::Polygon& area,
                            double grid_spacing_m = 1.0);
    void UseDefaultExplorationArea(double grid_spacing_m = 1.5);

    bool HasFrontier() const;
    bool SelectNextFrontier(commsgs::geometry_msgs::PoseStamped& goal);
    void MarkFrontierVisited();

    float exploration_progress() const;
    float explored_area_m2() const;

    void CancelActiveMotion();

    explicit ExplorationClient(navigation::NavigationClient::Ptr navigation);

private:
    navigation::NavigationClient::Ptr navigation_;
    std::vector<commsgs::geometry_msgs::PoseStamped> frontiers_;
    size_t next_frontier_index_{0};
    size_t visited_frontiers_{0};
    double grid_spacing_m_{1.5};
    std::string map_name_{"exploration_map"};
};

}  // namespace exploration
}  // namespace task
}  // namespace autonomy
