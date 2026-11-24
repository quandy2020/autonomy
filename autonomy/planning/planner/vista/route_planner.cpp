/* Copyright 2026 The Openbot Authors. */
/** @file route_planner.cpp
 * @brief Map-contract admission and visibility routes without partial graph exposure.
 */
#include "autonomy/planning/planner/vista/route_planner.hpp"
#include "autonomy/common/logging.hpp"
#include <algorithm>
#include <limits>
#include <queue>
#include <stdexcept>

namespace autonomy::planning::planner::vista {
namespace {
/** @brief Copy only evidence belonging to the locked graph/map pair. */
VistaMapEvidence CopyEvidence(const VisibilityGraph& graph, const std::string& frame) {
    return {graph.map_epoch, graph.geometry_revision, graph.observation_watermark,
        graph.observation_cut, graph.observation_baseline,
        graph.observation_recovery_generation, frame};
}
/** @brief Reject malformed views before replacing any accepted graph geometry. */
void ValidateView(const perception::MapReadView& view,
                  const std::optional<perception::MapReadView>& previous) {
    if (!view.grid || !view.map_epoch || !view.geometry_revision ||
        !std::isfinite(view.source_stamp) || view.frame_id.empty() ||
        view.frame_id != view.grid->frame_id())
        throw std::invalid_argument("invalid visibility map view");
    for (const auto& b : view.dirty_regions)
        if (!std::isfinite(b.min_x) || !std::isfinite(b.min_y) ||
            !std::isfinite(b.max_x) || !std::isfinite(b.max_y) ||
            b.min_x > b.max_x || b.min_y > b.max_y)
            throw std::invalid_argument("invalid visibility dirty region");
    if (!previous || previous->map_epoch != view.map_epoch) return;
    if (previous->frame_id != view.frame_id ||
        previous->footprint_inflated != view.footprint_inflated ||
        previous->grid->resolution() != view.grid->resolution() ||
        previous->has_height_evidence != view.has_height_evidence)
        throw std::invalid_argument("visibility geometry contract changed without a new epoch");
    const double resolution = view.grid->resolution();
    for (double offset : {(view.grid->origin_x() - previous->grid->origin_x()) / resolution,
                          (view.grid->origin_y() - previous->grid->origin_y()) / resolution})
        if (!std::isfinite(offset) || std::abs(offset - std::round(offset)) > 1e-6)
            throw std::invalid_argument("visibility grid lattice changed without a new epoch");
}
}  // namespace
namespace {
/** @brief Find a nearby strictly safe pose for a bounded route-start escape. */
std::optional<Pose2d> FindStartAnchor(const perception::MapReadView& map,
                                      const EdgeValidator& validator,
                                      const Pose2d& start) {
  if (!map.grid) return {};
  const auto center = map.grid->WorldToCell(start);
  if (!center) return {};
  const double resolution = map.grid->resolution();
  // A 0.30 m ring can be entirely inside the footprint band in a corner or
  // doorway, which left the route planner with no admissible origin at all.
  // The anchor only seeds the search; the command gate still certifies the
  // actual motion, so a bounded 0.60 m ring keeps the planner usable.
  const int span = std::max(1, static_cast<int>(std::ceil(.60 / resolution)));
  std::optional<Pose2d> best;
  double best_distance = std::numeric_limits<double>::infinity();
  for (int dy = -span; dy <= span; ++dy) for (int dx = -span; dx <= span; ++dx) {
    const int x = center->x + dx, y = center->y + dy;
    if (!map.grid->Contains(x, y) || map.grid->At(x, y).state !=
        autonomy::navigation::CellState::kFree) continue;
    auto candidate = map.grid->CellCenter(x, y);
    candidate.yaw = start.yaw;
    const double distance = autonomy::navigation::Distance(start, candidate);
    if (distance <= 1e-9 || distance > .60 + 1e-9 || distance >= best_distance) continue;
    if (!validator.PoseSafe(candidate)) continue;
    best = candidate; best_distance = distance;
  }
  return best;
}
}  // namespace

VistaRoutePlanner::VistaRoutePlanner(autonomy::navigation::SafetyOptions options, std::size_t nodes)
    : options_(options), updater_(nodes) {}
GraphUpdateStats VistaRoutePlanner::Update(const perception::MapReadView& map) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (map_ && (map.map_epoch < map_->map_epoch ||
        (map.map_epoch == map_->map_epoch &&
         map.geometry_revision < map_->geometry_revision))) return {};
    ValidateView(map, map_);
    // Stage allocating view ownership and safety construction before graph mutation.
    auto pending = map;
    EdgeValidator validator(map,options_);
    const bool changed_extent = map_ &&
        (map_->grid->origin_x() != map.grid->origin_x() ||
         map_->grid->origin_y() != map.grid->origin_y() ||
         map_->grid->width() != map.grid->width() || map_->grid->height() != map.grid->height());
    const bool full = !ready_ || changed_extent;
    try {
        auto stats = updater_.Update(map,validator,graph_,full);
        map_ = std::move(pending); ready_ = true; return stats;
    } catch (...) {
        // Retain only the admission contract, never route over a partial graph
        // or authorize old free space after receiving a newer blocked snapshot.
        graph_ = {}; updater_.Clear(); ready_ = false; throw;
    }
}
void VistaRoutePlanner::SetAllowUnknown(bool allow) {
    std::lock_guard<std::mutex> lock(mutex_);
    options_.allow_unknown = allow;
}
VisibilityGraph VistaRoutePlanner::GraphSnapshot() const {
    std::lock_guard<std::mutex> lock(mutex_); return graph_;
}
std::optional<VistaMapEvidence> VistaRoutePlanner::Evidence() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!ready_ || !map_) return {};
    return CopyEvidence(graph_, map_->frame_id);
}
VistaRoute VistaRoutePlanner::CreateRoute(const Pose2d& start, const Pose2d& goal,
                                         const std::function<bool()>& canceled) const {
    std::optional<perception::MapReadView> map;
    VisibilityGraph graph;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (ready_) { map = map_; graph = graph_; }
    }
    VistaRoute result;
    if (!map || !autonomy::navigation::IsFinite(start) || !autonomy::navigation::IsFinite(goal)) return result;
    result.map_epoch = map->map_epoch; result.geometry_revision = map->geometry_revision;
    result.path.frame_id = map->frame_id;
    result.evidence = CopyEvidence(graph, map->frame_id);
    auto is_canceled = [&] { return canceled && canceled(); };
    if (is_canceled()) { result.canceled = true; return result; }
    EdgeValidator validator(*map,options_);
    Pose2d route_start = start;
    if (!validator.PoseSafe(start)) {
        const auto check = validator.safety().VerifyPose(start);
        const auto cell = map->grid->WorldToCell(start);
        AINFO_EVERY(20) << "VISTA_ROUTE_START_UNSAFE x=" << start.x
                        << " y=" << start.y << " support="
                        << (map->startup_support ? 1 : 0)
                        << " reason=" << check.reason
                        << " clearance=" << check.clearance
                        << " cell=" << (cell ? static_cast<int>(map->grid->At(cell->x,cell->y).state) : -1)
                        << " footprint_inflated=" << (map->footprint_inflated ? 1 : 0)
                        << " map=" << map->grid->width() << "x" << map->grid->height()
                        << " origin=" << map->grid->origin_x() << "," << map->grid->origin_y();
        if (map->startup_support) {
            AINFO_EVERY(20) << "VISTA_ROUTE_SUPPORT center="
                            << map->startup_support->center_x << ","
                            << map->startup_support->center_y
                            << " radius=" << map->startup_support->radius
                            << " floor=" << map->startup_support->floor_height
                            << " ceiling=" << map->startup_support->ceiling_height;
        }
        // An occupied self-hit on the robot's own cell must not block routing:
        // a nearby strictly safe cell is a valid route origin.
        const auto anchor = FindStartAnchor(*map, validator, start);
        if (!anchor) return result;
        route_start = *anchor;
        AINFO_EVERY(20) << "VISTA_ROUTE_START_ESCAPE anchor="
                        << route_start.x << "," << route_start.y
                        << " distance=" << autonomy::navigation::Distance(start, route_start);
    }
    if (!validator.PoseSafe(goal)) {
        const auto check = validator.safety().VerifyPose(goal);
        AINFO_EVERY(20) << "VISTA_ROUTE_GOAL_UNSAFE x=" << goal.x
                        << " y=" << goal.y << " support="
                        << (map->startup_support ? 1 : 0)
                        << " reason=" << check.reason
                        << " clearance=" << check.clearance;
        // A 2D route may terminate inside the same bounded near-field deficit
        // that is admissible for the start; the command gate remains the
        // authority for actual motion. Unbounded obstacles still reject.
        if (!validator.StartEscapable(goal)) return result;
    }
    const NodeId source = 0, target = std::numeric_limits<NodeId>::max();
    std::map<NodeId,Pose2d> poses{{source,route_start},{target,goal}};
    std::map<NodeId,std::vector<std::pair<NodeId,double>>> adjacency;
    auto connect = [&](NodeId a, NodeId b, double length) {
        adjacency[a].push_back({b,length}); adjacency[b].push_back({a,length});
    };
    for (const auto& n : graph.nodes) poses[n.first] = n.second.pose;
    for (const auto& e : graph.edges) if (e.second.valid) connect(e.first.first,e.first.second,e.second.length);
    for (const auto& n : poses) {
        if (is_canceled()) { result.canceled = true; return result; }
        if (n.first != source && validator.Validate(source,route_start,n.first,n.second).valid)
            connect(source,n.first,autonomy::navigation::Distance(route_start,n.second));
        if (n.first != target && n.first != source && validator.Validate(target,goal,n.first,n.second).valid)
            connect(target,n.first,autonomy::navigation::Distance(goal,n.second));
    }
    using QueueEntry = std::pair<double,NodeId>;
    std::priority_queue<QueueEntry,std::vector<QueueEntry>,std::greater<QueueEntry>> queue;
    std::map<NodeId,double> distances; std::map<NodeId,NodeId> parent;
    distances[source] = 0; queue.push({0,source});
    while (!queue.empty()) {
        if (is_canceled()) { result.canceled = true; return result; }
        auto current = queue.top(); queue.pop();
        if (current.first != distances[current.second]) continue;
        if (current.second == target) break;
        for (const auto& next : adjacency[current.second]) {
            const double d = current.first+next.second;
            if (!distances.count(next.first) || d < distances[next.first]) {
                distances[next.first] = d; parent[next.first] = current.second; queue.push({d,next.first});
            }
        }
    }
    if (distances.count(target)) {
        NodeId n = target;
        while (n != source) { result.path.poses.push_back(poses.at(n)); n = parent.at(n); }
        result.path.poses.push_back(route_start); std::reverse(result.path.poses.begin(),result.path.poses.end());
        if (autonomy::navigation::Distance(start, route_start) > 1e-9) {
            result.path.poses.insert(result.path.poses.begin(), start);
            result.length += autonomy::navigation::Distance(start, route_start);
        }
        result.length += distances.at(target); result.success = true;
    } else {
        result.used_grid_fallback = true;
        // Grid connectivity is a fallback, so graph sparsification cannot declare unreachable.
        // The fallback must expand from the admitted route origin: the raw pose can
        // be inside its own safety margin and would yield an empty field.
        autonomy::navigation::ReachabilityField field(
            std::make_shared<autonomy::navigation::GeometrySafety>(validator.safety()),route_start,0,canceled);
        auto route = field.PathTo(goal);
        if (field.canceled() || is_canceled()) { result.canceled = true; return result; }
        if (!route.poses.empty()) {
            result.path = route; result.success = true;
            for (std::size_t i = 1; i < route.poses.size(); ++i)
                result.length += autonomy::navigation::Distance(route.poses[i-1],route.poses[i]);
        }
    }
    // A 2D route is geometrically checked, but 3D/dynamic Safety still owns execution permission.
    result.execution_verified = false;
    return result;
}
}  // namespace autonomy::planning::planner::vista
