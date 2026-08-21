/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/exploration/exploration_client.hpp"
#include "autonomy/task/behavior_tree/blackboard_client.hpp"

#include <fstream>

#include "autonomy/common/logging.hpp"
#include "autonomy/exploration/planner/exploration_planner.hpp"
#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_cpp/tree_node.h"

namespace autonomy {
namespace task {
namespace exploration {
ExplorationClient::ExplorationClient(
    navigation::NavigationClient::Ptr navigation)
    : navigation_(std::move(navigation))
{
    EnsureExplorer();
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
    BlackboardClientStore<ExplorationClient>::SetShared(client);
    if (client) {
        navigation::NavigationClient::SetShared(client->navigation_);
    }
}

ExplorationClient::Ptr ExplorationClient::FromBlackboard(
    const std::shared_ptr<BT::Blackboard>& blackboard)
{
    return BlackboardClientStore<ExplorationClient>::FromBlackboard(blackboard, kExplorationClientBlackboardKey);
}

ExplorationClient::Ptr ExplorationClient::FromNode(const BT::TreeNode& node)
{
    return BlackboardClientStore<ExplorationClient>::FromNode(node, kExplorationClientBlackboardKey);
}

void ExplorationClient::EnsureExplorer()
{
    if (!explorer_) {
        explorer_ =
            std::make_shared<::autonomy::exploration::planner::ExplorationPlanner>();
        explorer_->UseDefaultExplorationArea();
    }
}

void ExplorationClient::SetOptions(
    const ::autonomy::exploration::proto::ExplorationOptions& options)
{
    EnsureExplorer();
    explorer_->Configure(options, explorer_->GetName());
}

void ExplorationClient::SetMapName(const std::string& map_name)
{
    if (!map_name.empty()) {
        map_name_ = map_name;
    }
}

void ExplorationClient::SetExplorationArea(
    const automsgs::msgs::geometry_msgs::Polygon& area)
{
    EnsureExplorer();
    explorer_->SetExplorationArea(area);
    AINFO << "ExplorationClient: RGBD exploration area set ("
          << area.points_size() << " vertices)";
}

void ExplorationClient::UseDefaultExplorationArea()
{
    EnsureExplorer();
    explorer_->UseDefaultExplorationArea();
}

void ExplorationClient::UpdateOdometry(
    const automsgs::msgs::nav_msgs::Odometry& odom)
{
    EnsureExplorer();
    if (!odom.header().frame_id().empty()) {
        map_frame_ = odom.header().frame_id();
    }
    explorer_->UpdateOdometry(odom);
    explorer_->ExecutePlanningCycle();
}

void ExplorationClient::UpdateDepth(
    const automsgs::msgs::sensor_msgs::Image& depth,
    const automsgs::msgs::sensor_msgs::CameraInfo& info,
    const automsgs::msgs::geometry_msgs::Transform& map_t_camera)
{
    EnsureExplorer();
    explorer_->UpdateDepth(depth, info, map_t_camera);
    explorer_->ExecutePlanningCycle();
}

bool ExplorationClient::HasFrontier() const
{
    if (!explorer_) {
        return false;
    }
    if (explorer_->IsFinished()) {
        return false;
    }
    return explorer_->HasExplorationTarget() || explorer_->Progress() < 0.99f;
}

bool ExplorationClient::SelectNextFrontier(
    automsgs::msgs::geometry_msgs::PoseStamped& goal)
{
    EnsureExplorer();
    if (!explorer_->GetNextWaypoint(goal)) {
        return false;
    }
    if (goal.header().frame_id().empty()) {
        goal.mutable_header()->set_frame_id(map_frame_);
    }
    return true;
}

void ExplorationClient::MarkFrontierVisited()
{
    if (explorer_) {
        explorer_->MarkWaypointReached();
        explorer_->ExecutePlanningCycle();
    }
}

float ExplorationClient::exploration_progress() const
{
    return explorer_ ? explorer_->Progress() : 0.f;
}

float ExplorationClient::explored_area_m2() const
{
    return explorer_ ? explorer_->ExploredAreaM2() : 0.f;
}

bool ExplorationClient::IsExplorationFinished() const
{
    return explorer_ && explorer_->IsFinished();
}

automsgs::msgs::map_msgs::OccupancyGrid ExplorationClient::GetOccupancyGrid(
    const std::string& frame_id) const
{
    if (!explorer_) {
        return {};
    }
    return explorer_->GetOccupancyGrid(
        frame_id.empty() ? map_frame_ : frame_id);
}

bool ExplorationClient::SaveExplorationMap(const std::string& map_name) const
{
    const auto grid = GetOccupancyGrid(map_frame_);
    if (grid.data().empty()) {
        AWARN << "SaveExplorationMap: empty occupancy grid";
        return false;
    }
    const std::string path = map_name + ".pgm";
    std::ofstream out(path, std::ios::binary);
    if (!out) {
        AERROR << "SaveExplorationMap: cannot write " << path;
        return false;
    }
    out << "P5\n" << grid.info().width() << " " << grid.info().height() << "\n255\n";
    for (const auto cell : grid.data()) {
        const unsigned char v =
            cell < 0 ? 205 : (cell > 50 ? 0 : 254);
        out.write(reinterpret_cast<const char*>(&v), 1);
    }
    AINFO << "SaveExplorationMap: wrote " << path << " ("
          << grid.info().width() << "x" << grid.info().height() << ")";
    return true;
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
