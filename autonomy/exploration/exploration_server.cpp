/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "autonomy/exploration/exploration_server.hpp"

#include <chrono>

#include "autonomy/common/logging.hpp"
#include "autonomy/exploration/constants.hpp"
#include "autonomy/exploration/planner/exploration_planner.hpp"

namespace autonomy {
namespace exploration {
namespace {

using SteadyClock = std::chrono::steady_clock;

double ReplanPeriodSec(const proto::ExplorationOptions& options)
{
    if (options.replan_min_period_sec() > 0.0) {
        return options.replan_min_period_sec();
    }
    if (options.planner_frequency() > 0.0) {
        return 1.0 / options.planner_frequency();
    }
    return 1.0;
}

}  // namespace

ExplorationServer::ExplorationServer(const proto::ExplorationOptions& options)
    : options_(options)
{
    LoadExplorers();
}

ExplorationServer::~ExplorationServer()
{
    Shutdown();
}

void ExplorationServer::LoadExplorers()
{
    explorers_.clear();
    active_explorer_id_ = options_.default_explorer().empty()
                              ? "rgbd_hierarchical"
                              : options_.default_explorer();

    auto add_rgbd = [&](const std::string& id) {
        auto explorer = std::make_shared<planner::ExplorationPlanner>(
            options_, id);
        explorers_[id] = explorer;
    };

    if (options_.explorer_plugins().empty()) {
        add_rgbd("rgbd_hierarchical");
        return;
    }
    for (const auto& plugin : options_.explorer_plugins()) {
        // Currently only rgbd hierarchical explorer is available in-process.
        add_rgbd(plugin.empty() ? "rgbd_hierarchical" : plugin);
    }
    if (explorers_.find(active_explorer_id_) == explorers_.end() &&
        !explorers_.empty()) {
        active_explorer_id_ = explorers_.begin()->first;
    }
}

void ExplorationServer::Start()
{
    if (started_) {
        return;
    }
    // ROS / embedded callers may not have run autolink::Init(); skip Autolink
    // node creation in that case and still mark the server ready.
    if (!autolink::OK()) {
        started_ = true;
        AINFO << "ExplorationServer started embedded (no Autolink; "
              << explorers_.size() << " explorer plugin(s), active="
              << active_explorer_id_ << ")";
        return;
    }
    node_ = autolink::CreateNode(kExplorationServerNodeName);
    AttachAutolinkNode(node_);
    started_ = true;
    AINFO << "ExplorationServer started (" << explorers_.size()
          << " explorer plugin(s), active=" << active_explorer_id_ << ")";
}

void ExplorationServer::Shutdown()
{
    DetachAutolinkNode();
    node_.reset();
    started_ = false;
}

bool ExplorationServer::AttachAutolinkNode(
    std::shared_ptr<autolink::Node> node)
{
    if (!node) {
        return false;
    }
    node_ = std::move(node);
    autolink_actions_ = std::make_unique<AutolinkActionServers>();
    AINFO << "ExplorationServer autolink node attached.";
    return true;
}

void ExplorationServer::DetachAutolinkNode()
{
    autolink_actions_.reset();
}

bool ExplorationServer::FindExplorerId(const std::string& requested,
                                       std::string* name) const
{
    if (!name) {
        return false;
    }
    if (!requested.empty() && explorers_.count(requested) > 0) {
        *name = requested;
        return true;
    }
    if (explorers_.count(active_explorer_id_) > 0) {
        *name = active_explorer_id_;
        return true;
    }
    return false;
}

common::ExplorerInterface::SharedPtr ExplorationServer::ActiveExplorer() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    std::string id;
    if (!FindExplorerId(active_explorer_id_, &id)) {
        return nullptr;
    }
    return explorers_.at(id);
}

void ExplorationServer::UpdateOdometry(
    const automsgs::msgs::nav_msgs::Odometry& odom)
{
    auto explorer = ActiveExplorer();
    if (!explorer) {
        return;
    }
    explorer->UpdateOdometry(odom);
    ComputeExploration();
}

void ExplorationServer::UpdateDepth(
    const automsgs::msgs::sensor_msgs::Image& depth,
    const automsgs::msgs::sensor_msgs::CameraInfo& info,
    const automsgs::msgs::geometry_msgs::Transform& map_t_camera)
{
    auto explorer = ActiveExplorer();
    if (!explorer) {
        return;
    }
    explorer->UpdateDepth(depth, info, map_t_camera);
    ComputeExploration();
}

void ExplorationServer::SetExplorationArea(
    const automsgs::msgs::geometry_msgs::Polygon& area)
{
    auto explorer = ActiveExplorer();
    if (explorer) {
        explorer->SetExplorationArea(area);
    }
}

void ExplorationServer::ComputeExploration()
{
    auto explorer = ActiveExplorer();
    if (!explorer) {
        return;
    }

    const double period_sec = ReplanPeriodSec(options_);
    const auto now = SteadyClock::now();
    if (has_last_plan_time_) {
        const double elapsed =
            std::chrono::duration<double>(now - last_plan_time_).count();
        if (elapsed < period_sec) {
            return;
        }
    }
    last_plan_time_ = now;
    has_last_plan_time_ = true;
    explorer->ExecutePlanningCycle();
}

bool ExplorationServer::GetNextWaypoint(
    automsgs::msgs::geometry_msgs::PoseStamped& out)
{
    auto explorer = ActiveExplorer();
    return explorer && explorer->GetNextWaypoint(out);
}

void ExplorationServer::MarkWaypointReached()
{
    auto explorer = ActiveExplorer();
    if (explorer) {
        explorer->MarkWaypointReached();
    }
}

bool ExplorationServer::HasExplorationTarget() const
{
    auto explorer = ActiveExplorer();
    return explorer && explorer->HasExplorationTarget();
}

bool ExplorationServer::IsFinished() const
{
    auto explorer = ActiveExplorer();
    return explorer && explorer->IsFinished();
}

float ExplorationServer::Progress() const
{
    auto explorer = ActiveExplorer();
    return explorer ? explorer->Progress() : 0.f;
}

float ExplorationServer::ExploredAreaM2() const
{
    auto explorer = ActiveExplorer();
    return explorer ? explorer->ExploredAreaM2() : 0.f;
}

}  // namespace exploration
}  // namespace autonomy
