/* Copyright 2026 The Openbot Authors. */
/** @file vista_planner.cpp
 * @brief Native configuration, immutable map reads and versioned route commits.
 */
#include "autonomy/planning/planner/vista/vista_planner.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include "autolink/plugin_manager/plugin_manager.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"

namespace autonomy::planning::planner::vista {
namespace {
bool PoseValid(const automsgs::msgs::geometry_msgs::PoseStamped& pose) {
    const auto& p = pose.pose().position(); const auto& q = pose.pose().orientation();
    const double norm = q.x()*q.x()+q.y()*q.y()+q.z()*q.z()+q.w()*q.w();
    return std::isfinite(p.x()) && std::isfinite(p.y()) && std::isfinite(p.z()) &&
           std::isfinite(norm) && norm > 1e-12;
}
Pose2d PlanarPose(const automsgs::msgs::geometry_msgs::PoseStamped& pose) {
    const auto& p = pose.pose().position(); const auto& q = pose.pose().orientation();
    const double norm = q.x()*q.x()+q.y()*q.y()+q.z()*q.z()+q.w()*q.w();
    return {p.x(),p.y(),std::atan2(2*(q.w()*q.z()+q.x()*q.y()),
                                  norm-2*(q.y()*q.y()+q.z()*q.z()))};
}
}  // namespace

VistaPlannerSettings ReadVistaPlannerSettings(const proto::VistaPlannerOptions& options) {
    VistaPlannerSettings settings;
    if (options.has_maximum_nodes()) settings.maximum_nodes = options.maximum_nodes();
    if (options.has_maximum_cells()) {
        if (options.maximum_cells() > std::numeric_limits<std::size_t>::max())
            throw std::length_error("Vista map capacity exceeds addressable size");
        settings.maximum_cells = static_cast<std::size_t>(options.maximum_cells());
    }
    if (!settings.maximum_nodes || !settings.maximum_cells)
        throw std::invalid_argument("Vista capacities must be positive");
    if (options.has_horizontal_margin()) settings.safety.horizontal_margin = options.horizontal_margin();
    if (options.has_robot_height()) settings.safety.robot_height = options.robot_height();
    if (options.has_vertical_margin()) settings.safety.vertical_margin = options.vertical_margin();
    if (options.has_max_floor_step()) settings.safety.max_floor_step = options.max_floor_step();
    if (options.has_costmap_footprint_inflated())
        settings.costmap_footprint_inflated = options.costmap_footprint_inflated();
    auto validation_map = std::make_shared<autonomy::navigation::GridMap>(1, 1, 1.0);
    autonomy::navigation::GeometrySafety validation(validation_map, settings.safety);
    return settings;
}

VistaPlanner::VistaPlanner(const proto::PlannerOptions& options, const std::string& name,
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap)
    : VistaPlanner(options, name, std::move(costmap), ReadVistaPlannerSettings(options.vista())) {}

VistaPlanner::VistaPlanner(const proto::PlannerOptions& options, const std::string& name,
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap, VistaPlannerSettings settings)
    : GlobalPlanner(options,name,std::move(costmap)) {
    if (!costmap_) throw std::invalid_argument("VistaPlanner needs a native Costmap");
    if (costmap_->getUseRadius()) settings.safety.robot_radius = costmap_->getRobotRadius();
    else {
        double radius = 0;
        for (const auto& p : costmap_->getRobotFootprint())
            radius = std::max(radius,std::hypot(p.x(),p.y()));
        if (radius > 0) settings.safety.robot_radius = radius;
    }
    Initialize(settings);
}

void VistaPlanner::Initialize(VistaPlannerSettings settings) {
    if (route_planner_) throw std::logic_error("VistaPlanner already configured");
    // Validate the actual safety values, independently from a preinflated map.
    auto validation_map = std::make_shared<autonomy::navigation::GridMap>(1,1,1.0);
    autonomy::navigation::GeometrySafety check(validation_map,settings.safety);
    auto routes = std::make_shared<VistaRoutePlanner>(settings.safety,settings.maximum_nodes);
    auto reader = std::make_shared<NativeCostmapReader>(settings.maximum_cells);
    settings_ = settings; route_planner_ = std::move(routes); costmap_reader_ = std::move(reader);
    ++configuration_version_;
}

void VistaPlanner::Configure(const proto::PlannerOptions& options, const std::string& name,
                             MapViewSource source, VistaPlannerSettings settings) {
    if (!source || name.empty()) throw std::invalid_argument("invalid VistaPlanner configuration");
    std::lock_guard<std::mutex> lock(mutex_);
    // Configure() is the path used by the planner server, and it used to skip
    // the footprint-derived radius applied by the constructor.  The route model
    // then required 0.20 m while exploration certified goals at 0.105 m, so a
    // robot standing 0.11 m from a wall could never obtain a route.  Adopt the
    // same body radius here so both layers agree on one footprint.
    if (costmap_) {
        if (costmap_->getUseRadius()) settings.safety.robot_radius = costmap_->getRobotRadius();
        else {
            double radius = 0;
            for (const auto& p : costmap_->getRobotFootprint())
                radius = std::max(radius, std::hypot(p.x(), p.y()));
            if (radius > 0) settings.safety.robot_radius = radius;
        }
    }
    AINFO << "VISTA_ROUTE_LIMITS robot_radius=" << settings.safety.robot_radius
          << " horizontal_margin=" << settings.safety.horizontal_margin
          << " required=" << settings.safety.robot_radius + settings.safety.horizontal_margin;
    Initialize(settings); options_ = options; name_ = name; map_source_ = std::move(source);
}
void VistaPlanner::SetMapViewSource(MapViewSource source) {
    if (!source) throw std::invalid_argument("Vista map source is empty");
    std::lock_guard<std::mutex> lock(mutex_);
    if (!route_planner_) throw std::logic_error("VistaPlanner is not configured");
    map_source_ = std::move(source);
    ++configuration_version_;
}
void VistaPlanner::SetCostmapReadMetadata(CostmapReadMetadata metadata) {
    if (!metadata.alignment_id || !std::isfinite(metadata.observation_source_stamp) ||
        metadata.observation_source_stamp < 0)
        throw std::invalid_argument("invalid Vista Costmap metadata");
    std::lock_guard<std::mutex> lock(mutex_); metadata_ = metadata; ++configuration_version_;
}
std::shared_ptr<VistaRoutePlanner> VistaPlanner::SharedRoutePlanner() const {
    std::lock_guard<std::mutex> lock(mutex_); return route_planner_;
}
std::optional<perception::MapReadView> VistaPlanner::LatestMapView() const {
    std::lock_guard<std::mutex> lock(mutex_); return latest_map_;
}
std::optional<VistaRoute> VistaPlanner::LastRoute() const {
    std::lock_guard<std::mutex> lock(mutex_); return last_route_;
}

uint32 VistaPlanner::QueryRoute(const automsgs::msgs::geometry_msgs::PoseStamped& start,
    const automsgs::msgs::geometry_msgs::PoseStamped& goal, VistaRoute& route,
    bool evidence_only, std::function<bool()> cancel_checker) {
    route = {};
    MapViewSource source;
    std::shared_ptr<VistaRoutePlanner> routes;
    std::shared_ptr<NativeCostmapReader> reader;
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap;
    VistaPlannerSettings settings;
    CostmapReadMetadata metadata;
    std::uint64_t configuration;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        configuration = configuration_version_;
        source = map_source_; routes = route_planner_; reader = costmap_reader_;
        costmap = costmap_; settings = settings_; metadata = metadata_;
    }
    auto canceled = [&] { return cancel_checker && cancel_checker(); };
    try {
        if (canceled()) return proto::PLANNER_CANCELED;
        if (!routes) return proto::PLANNER_NOT_INITIALIZED;
        if (!evidence_only && !PoseValid(start)) return proto::PLANNER_INVALID_START;
        if (!evidence_only && !PoseValid(goal)) return proto::PLANNER_INVALID_GOAL;
        std::optional<perception::MapReadView> view;
        if (source) view = source();
        else if (costmap && costmap->getCostmap())
            view = reader->Read(*costmap->getCostmap(), costmap->getGlobalFrameID(),
                                metadata, settings.costmap_footprint_inflated);
        if (canceled()) return proto::PLANNER_CANCELED;
        if (!view || !view->grid || !view->map_epoch || !view->geometry_revision)
            return proto::PLANNER_NOT_INITIALIZED;
        if (view->frame_id != view->grid->frame_id() || (!evidence_only &&
            (start.header().frame_id() != view->frame_id || goal.header().frame_id() != view->frame_id)))
            return proto::PLANNER_TF_ERROR;
        routes->Update(*view);
        const auto evidence = routes->Evidence();
        if (!evidence || evidence->map_epoch != view->map_epoch ||
            evidence->geometry_revision != view->geometry_revision)
            return proto::PLANNER_TF_ERROR;
        if (!(evidence->observation_cut == view->observation_cut) ||
            !(evidence->observation_baseline == view->observation_baseline) ||
            evidence->observation_recovery_generation != view->observation_recovery_generation)
            return proto::PLANNER_CANCELED;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (configuration != configuration_version_) return proto::PLANNER_CANCELED;
            latest_map_ = *view;
        }
        route.evidence = evidence;
        route.map_epoch = evidence->map_epoch;
        route.geometry_revision = evidence->geometry_revision;
        route.path.frame_id = evidence->frame_id;
        if (evidence_only) {
            // A provenance refresh never manufactures a successful route.
        } else {
            const Pose2d from = PlanarPose(start), to = PlanarPose(goal);
            if (!view->grid->WorldToCell(from)) return proto::PLANNER_INVALID_START;
            if (!view->grid->WorldToCell(to)) return proto::PLANNER_INVALID_GOAL;
            EdgeValidator validator(*view, settings.safety);
            if (!validator.StartEscapable(from)) return proto::PLANNER_BLOCKED_START;
            if (!validator.PoseSafe(to)) return proto::PLANNER_BLOCKED_GOAL;
            route = routes->CreateRoute(from, to, canceled);
            if (route.canceled || canceled()) return proto::PLANNER_CANCELED;
            if (!route.success) return proto::PLANNER_NO_PATH_FOUND;
            if (route.path.poses.empty()) return proto::PLANNER_EMPTY_PATH;
        }
        if (canceled()) { route = {}; route.canceled = true; return proto::PLANNER_CANCELED; }
        const auto current = routes->Evidence();
        if (!current || !route.evidence || current->map_epoch != route.map_epoch ||
            current->geometry_revision != route.geometry_revision ||
            !(current->observation_cut == route.evidence->observation_cut) ||
            !(current->observation_baseline == route.evidence->observation_baseline) ||
            current->observation_recovery_generation != route.evidence->observation_recovery_generation) {
            route = {}; return proto::PLANNER_CANCELED;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        if (configuration != configuration_version_) { route = {}; return proto::PLANNER_CANCELED; }
        return proto::PLANNER_SUCCESS;
    } catch (...) {
        route = {}; return proto::PLANNER_INTERNAL_ERROR;
    }
}

uint32 VistaPlanner::CreatePlan(const automsgs::msgs::geometry_msgs::PoseStamped& start,
    const automsgs::msgs::geometry_msgs::PoseStamped& goal,
    automsgs::msgs::nav_msgs::Path& plan, std::function<bool()> cancel_checker) {
    plan.Clear();
    std::uint64_t request, configuration;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        request = ++latest_request_; configuration = configuration_version_;
        last_route_.reset();
    }
    auto canceled = [&] { return cancel_checker && cancel_checker(); };
    try {
        VistaRoute route;
        const auto code = QueryRoute(start, goal, route, false, cancel_checker);
        if (code != proto::PLANNER_SUCCESS) return code;
        automsgs::msgs::nav_msgs::Path candidate;
        candidate.mutable_header()->CopyFrom(start.header());
        candidate.mutable_header()->set_frame_id(route.path.frame_id);
        for (std::size_t i = 0; i < route.path.poses.size(); ++i) {
            auto* pose = candidate.add_poses(); pose->mutable_header()->CopyFrom(candidate.header());
            auto* p = pose->mutable_pose()->mutable_position();
            p->set_x(route.path.poses[i].x); p->set_y(route.path.poses[i].y);
            // This plugin plans in a single planar map. It does not infer floor
            // height or overhead clearance from a 2D Costmap.
            p->set_z(start.pose().position().z());
            auto* q = pose->mutable_pose()->mutable_orientation();
            double yaw = route.path.poses[i].yaw;
            if (i+1 < route.path.poses.size()) yaw = std::atan2(
                route.path.poses[i+1].y-route.path.poses[i].y,
                route.path.poses[i+1].x-route.path.poses[i].x);
            q->set_z(std::sin(yaw/2)); q->set_w(std::cos(yaw/2));
        }
        candidate.mutable_poses(0)->mutable_pose()->CopyFrom(start.pose());
        candidate.mutable_poses(candidate.poses_size()-1)->mutable_pose()->CopyFrom(goal.pose());
        if (canceled()) return proto::PLANNER_CANCELED;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (request != latest_request_ || configuration != configuration_version_)
                return proto::PLANNER_CANCELED;
            const auto current = route_planner_->Evidence();
            if (!current || !route.evidence || route.map_epoch != current->map_epoch ||
                route.geometry_revision != current->geometry_revision ||
                !(route.evidence->observation_cut == current->observation_cut) ||
                !(route.evidence->observation_baseline == current->observation_baseline) ||
                route.evidence->observation_recovery_generation != current->observation_recovery_generation)
                return proto::PLANNER_CANCELED;
            last_route_ = std::move(route); plan.Swap(&candidate);
        }
        return proto::PLANNER_SUCCESS;
    } catch (...) {
        plan.Clear();
        std::lock_guard<std::mutex> lock(mutex_);
        if (request == latest_request_) last_route_.reset();
        return proto::PLANNER_INTERNAL_ERROR;
    }
}

}  // namespace autonomy::planning::planner::vista

using autonomy::planning::planner::vista::VistaPlanner;
using autonomy::planning::common::GlobalPlanner;
AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(VistaPlanner, GlobalPlanner);
