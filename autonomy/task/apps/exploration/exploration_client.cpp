/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/apps/exploration/exploration_client.hpp"

#include <algorithm>
#include <cmath>

#include "autonomy/common/logging.hpp"
#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_cpp/tree_node.h"

namespace autonomy {
namespace task {
namespace exploration {
namespace {

std::weak_ptr<ExplorationClient> g_shared_client;

constexpr char kDefaultMapFrame[] = "map";
constexpr double kDefaultAreaHalfExtent = 5.0;

bool PointInPolygon(double x, double y,
                    const commsgs::geometry_msgs::Polygon& polygon)
{
    const auto& points = polygon.points;
    if (points.size() < 3) {
        return false;
    }

    bool inside = false;
    for (size_t i = 0, j = points.size() - 1; i < points.size(); j = i++) {
        const bool intersect =
            ((points[i].y > y) != (points[j].y > y)) &&
            (x < (points[j].x - points[i].x) * (y - points[i].y) /
                         (points[j].y - points[i].y + 1e-12) +
                     points[i].x);
        if (intersect) {
            inside = !inside;
        }
    }
    return inside;
}

commsgs::geometry_msgs::PoseStamped MakeFrontierPose(double x, double y)
{
    commsgs::geometry_msgs::PoseStamped pose;
    pose.header.frame_id = kDefaultMapFrame;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.orientation.w = 1.0;
    return pose;
}

void AppendGridFrontiers(const commsgs::geometry_msgs::Polygon& area,
                         double spacing,
                         std::vector<commsgs::geometry_msgs::PoseStamped>* out)
{
    if (area.points.empty() || spacing <= 0.0) {
        return;
    }

    double min_x = area.points.front().x;
    double max_x = area.points.front().x;
    double min_y = area.points.front().y;
    double max_y = area.points.front().y;
    for (const auto& point : area.points) {
        min_x = std::min(min_x, static_cast<double>(point.x));
        max_x = std::max(max_x, static_cast<double>(point.x));
        min_y = std::min(min_y, static_cast<double>(point.y));
        max_y = std::max(max_y, static_cast<double>(point.y));
    }

    for (double y = min_y; y <= max_y + 1e-6; y += spacing) {
        for (double x = min_x; x <= max_x + 1e-6; x += spacing) {
            if (!PointInPolygon(x, y, area)) {
                continue;
            }
            out->push_back(MakeFrontierPose(x, y));
        }
    }
}

commsgs::geometry_msgs::Polygon DefaultExplorationPolygon()
{
    commsgs::geometry_msgs::Polygon area;
    const double h = kDefaultAreaHalfExtent;
    area.points = {
        {static_cast<float>(-h), static_cast<float>(-h), 0.f},
        {static_cast<float>(h), static_cast<float>(-h), 0.f},
        {static_cast<float>(h), static_cast<float>(h), 0.f},
        {static_cast<float>(-h), static_cast<float>(h), 0.f},
    };
    return area;
}

}  // namespace

ExplorationClient::ExplorationClient(navigation::NavigationClient::Ptr navigation)
    : navigation_(std::move(navigation))
{
}

ExplorationClient::Ptr ExplorationClient::Create(
    navigation::NavigationClient::Ptr navigation)
{
    if (!navigation) {
        return nullptr;
    }
    return std::make_shared<ExplorationClient>(std::move(navigation));
}

ExplorationClient::Ptr ExplorationClient::Create(
    std::shared_ptr<autolink::Node> node)
{
    return Create(navigation::NavigationClient::Create(std::move(node)));
}

void ExplorationClient::SetShared(const Ptr& client)
{
    g_shared_client = client;
    if (client) {
        navigation::NavigationClient::SetShared(client->navigation_);
    }
}

ExplorationClient::Ptr ExplorationClient::FromBlackboard(
    const std::shared_ptr<BT::Blackboard>& blackboard)
{
    if (blackboard) {
        Ptr client;
        if (blackboard->get(kExplorationClientBlackboardKey, client) && client) {
            return client;
        }
    }
    return g_shared_client.lock();
}

ExplorationClient::Ptr ExplorationClient::FromNode(const BT::TreeNode& node)
{
    return FromBlackboard(node.config().blackboard);
}

void ExplorationClient::SetMapName(const std::string& map_name)
{
    if (!map_name.empty()) {
        map_name_ = map_name;
    }
}

void ExplorationClient::SetExplorationArea(
    const commsgs::geometry_msgs::Polygon& area, double grid_spacing_m)
{
    grid_spacing_m_ = grid_spacing_m;
    frontiers_.clear();
    next_frontier_index_ = 0;
    visited_frontiers_ = 0;
    AppendGridFrontiers(area, grid_spacing_m_, &frontiers_);
    AINFO << "ExplorationClient: " << frontiers_.size()
          << " frontier(s) in exploration area";
}

void ExplorationClient::UseDefaultExplorationArea(double grid_spacing_m)
{
    SetExplorationArea(DefaultExplorationPolygon(), grid_spacing_m);
}

bool ExplorationClient::HasFrontier() const
{
    return next_frontier_index_ < frontiers_.size();
}

bool ExplorationClient::SelectNextFrontier(
    commsgs::geometry_msgs::PoseStamped& goal)
{
    if (!HasFrontier()) {
        return false;
    }
    goal = frontiers_[next_frontier_index_++];
    return true;
}

void ExplorationClient::MarkFrontierVisited()
{
    ++visited_frontiers_;
}

float ExplorationClient::exploration_progress() const
{
    if (frontiers_.empty()) {
        return 0.f;
    }
    return static_cast<float>(visited_frontiers_) /
           static_cast<float>(frontiers_.size());
}

float ExplorationClient::explored_area_m2() const
{
    const double cell = grid_spacing_m_ * grid_spacing_m_;
    return static_cast<float>(visited_frontiers_ * cell);
}

void ExplorationClient::CancelActiveMotion()
{
    if (navigation_) {
        navigation_->CancelActiveMotion();
    }
}

}  // namespace exploration
}  // namespace task
}  // namespace autonomy
