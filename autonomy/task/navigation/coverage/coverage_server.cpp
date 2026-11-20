// Copyright (c) 2023 Open Navigation LLC; Autonomy adaptation 2026.
// SPDX-License-Identifier: Apache-2.0
/** @file coverage_server.cpp
 * @brief Native complete-coverage coverage server implementation. */
#include "autonomy/task/navigation/coverage/coverage_server.hpp"

#include "autonomy/task/navigation/coverage/map_connector.hpp"
#include "autonomy/task/navigation/coverage/region_orderer.hpp"
#include "autonomy/task/navigation/coverage/region_segmenter.hpp"

#include <boost/geometry.hpp>
#include <cmath>
#include <limits>

#include "autonomy/planning/planner/fields2cover/path_planning/constrained_connection.hpp"

namespace autonomy::task::coverage {
namespace {
void Validate(const F2CCells& cells) {
  using BPoint = boost::geometry::model::d2::point_xy<double>;
  using BPolygon = boost::geometry::model::polygon<BPoint>;
  if (cells.IsEmpty() || cells.Area() < 1e-6)
    throw RequestException("Empty field");
  size_t count = 0;
  for (const auto& cell : cells) {
    for (const auto& ring : cell)
      for (const auto& p : ring) {
        if (!std::isfinite(p.GetX()) || !std::isfinite(p.GetY()))
          throw RequestException("Nonfinite coordinate");
        if (++count > 4096)
          throw RequestException("Field has more than 4096 vertices");
      }
    BPolygon polygon;
    boost::geometry::read_wkt(cell.ExportToWkt(), polygon);
    boost::geometry::correct(polygon);
    std::string reason;
    if (!boost::geometry::is_valid(polygon, reason))
      throw RequestException("Invalid field topology: " + reason);
  }
}
F2CPoint OutputPoint(const F2CPoint& point, const F2CField& field,
                     bool cartesian) {
  auto result = point + field.GetRefPoint();
  if (!cartesian)
    result =
        f2c::Transform::transform(result, field.GetCRS(), field.GetPrevCRS());
  return result;
}
void Pose(NavPath& path, const F2CPoint& point, double angle,
          const std::string& frame) {
  auto* pose = path.add_poses();
  pose->mutable_header()->set_frame_id(frame);
  auto* p = pose->mutable_pose()->mutable_position();
  p->set_x(point.GetX());
  p->set_y(point.GetY());
  p->set_z(point.GetZ());
  auto* q = pose->mutable_pose()->mutable_orientation();
  q->set_z(std::sin(angle / 2));
  q->set_w(std::cos(angle / 2));
}
using CleanPoint = std::pair<double, double>;

bool NearlyEqual(const CleanPoint& a, const CleanPoint& b) {
  return std::hypot(a.first - b.first, a.second - b.second) < 1e-8;
}

bool IsSpike(const CleanPoint& previous, const CleanPoint& current,
             const CleanPoint& next) {
  const double ax = previous.first - current.first;
  const double ay = previous.second - current.second;
  const double bx = next.first - current.first;
  const double by = next.second - current.second;
  const double cross = ax * by - ay * bx;
  const double dot = ax * bx + ay * by;
  return std::abs(cross) < 1e-9 && dot < 0.0;
}

std::vector<CleanPoint> CleanRing(const F2CLinearRing& ring) {
  std::vector<CleanPoint> points;
  for (const auto& point : ring) {
    const CleanPoint value{point.GetX(), point.GetY()};
    if (points.empty() || !NearlyEqual(points.back(), value)) {
      points.push_back(value);
    }
  }
  if (points.size() > 1 && NearlyEqual(points.front(), points.back())) {
    points.pop_back();
  }
  bool changed = true;
  while (changed && points.size() > 3) {
    changed = false;
    for (std::size_t index = 0; index < points.size(); ++index) {
      const auto& previous =
          points[(index + points.size() - 1) % points.size()];
      const auto& current = points[index];
      const auto& next = points[(index + 1) % points.size()];
      if (IsSpike(previous, current, next)) {
        points.erase(points.begin() + static_cast<std::ptrdiff_t>(index));
        changed = true;
        break;
      }
    }
  }
  double signed_area = 0.0;
  for (std::size_t index = 0; index < points.size(); ++index) {
    const auto& current = points[index];
    const auto& next = points[(index + 1) % points.size()];
    signed_area += current.first * next.second - next.first * current.second;
  }
  if (signed_area < 0.0) {
    std::reverse(points.begin(), points.end());
  }
  return points;
}

F2CSwaths Flatten(const F2CRoute& route) {
  F2CSwaths result;
  for (const auto& group : route.GetVectorSwaths()) result.Append(group);
  return result;
}

/** @brief Coverage generated for a single room. */
struct RoomCoverage {
  F2CPath path;
  F2CSwaths swaths;
  double coverage{0.0};
  int swath_count{0};
};

/**
 * @brief Plan headland, swaths, route and path inside one room.
 * @param cells Room polygon in the request frame.
 * @param config Effective planner configuration.
 * @param goal Original request carrying the swath, route and path modes.
 * @param headland Whether the room keeps a perimeter headland band.
 * @return Planned path, ordered swaths and achieved coverage.
 */
RoomCoverage PlanRoom(const F2CCells& cells, const Configuration& config,
                      const proto::CoverageRequest& goal, bool headland) {
  RoomCoverage output;
  if (cells.IsEmpty() || cells.Area() < 1e-3) {
    return output;
  }
  HeadlandGenerator headlands(config);
  F2CCells core = headland
                      ? headlands.GenerateHeadlands(cells, goal.headland_mode())
                      : cells.Clone();
  if (core.IsEmpty() || core.Area() < 1e-3) {
    return output;
  }
  SwathGenerator swath_generator(config);
  auto grouped = swath_generator.GenerateSwathsByCells(core, goal.swath_mode());
  if (grouped.SizeTotal() == 0) {
    return output;
  }
  RouteGenerator route_generator(config);
  F2CRoute route =
      route_generator.GenerateRoute(core, core, grouped, goal.route_mode(), {});
  route.SetField(cells);
  F2CSwaths ordered = Flatten(route);
  if (ordered.size() != grouped.SizeTotal()) {
    return output;
  }
  PathGenerator path_generator(config);
  F2CPath path = path_generator.GeneratePath(route, goal.path_mode());
  if (path.size() < 2) {
    return output;
  }
  // Sweep the wall band as well: the interior swaths stop at the
  // headland, so a perimeter loop is what reaches the room boundary.
  F2CPath combined;
  if (headland) {
    for (const auto& band : headlands.GenerateHeadlandSwaths(
             cells, config.operation_width, goal.headland_mode())) {
      F2CPath loop = path_generator.GeneratePerimeter(
          band, cells, path.AtStart(), goal.path_mode());
      if (loop.size() > 1) {
        combined += loop;
      }
    }
  }
  const bool perimeter = combined.size() > 1;
  combined += path;
  output.path = combined;
  output.swaths = ordered;
  output.swath_count = static_cast<int>(ordered.size());
  const double core_coverage =
      f2c::obj::FieldCoverage().ComputeCost(core, ordered);
  const double room_area = cells.Area();
  const double band_area = std::max(0.0, room_area - core.Area());
  output.coverage =
      room_area > 1e-9
          ? (core_coverage * core.Area() + (perimeter ? band_area : 0.0)) /
                room_area
          : core_coverage;
  return output;
}

/**
 * @brief Plan every room of an occupancy map and concatenate one global path.
 *
 * Each room is planned independently so that turn connections never have to
 * cross a wall, then the room paths are joined in nearest-neighbour/2-opt
 * order. The result carries per-room polygons, paths and coverage plus the
 * stitched global path consumed by the visualizer.
 */
ComputeAction::Result PlanMapRegions(
    const Configuration& config, const proto::CoverageRequest& goal,
    const F2CField& master, const F2CCells& field, const std::string& frame,
    const CancelChecker& cancel,
    const std::function<void(const std::string&, double)>& stage) {
  ComputeAction::Result result;
  const bool headland =
      Enabled(goal.has_generate_headland(), goal.generate_headland());
  const double radius =
      goal.has_robot_radius() ? goal.robot_radius() : config.robot_radius;
  MapConnector::Options connector_options;
  if (goal.map().occupied_threshold() > 0) {
    connector_options.occupied_threshold = goal.map().occupied_threshold();
  }
  connector_options.clearance = std::max(0.0, radius);
  const MapConnector connector(goal.map(), connector_options);
  if (!connector.Valid()) {
    throw RequestException("Occupancy map has no traversable free space");
  }
  std::vector<F2CCells> rooms = RegionSegmenter::MapSplit(goal.map(), {});
  if (rooms.empty()) {
    throw RequestException("Map split found no rooms");
  }

  // One anchor per room: the traversable cell closest to the room centroid.
  // Room-to-room costs are navigable distances between anchors, so the tour
  // never assumes a straight line through a wall.
  std::vector<std::pair<double, double>> anchors;
  anchors.reserve(rooms.size());
  for (const auto& cells : rooms) {
    double sum_x = 0.0;
    double sum_y = 0.0;
    int count = 0;
    if (!cells.IsEmpty()) {
      for (const auto& point : cells.GetCellBorder(0)) {
        sum_x += point.GetX();
        sum_y += point.GetY();
        ++count;
      }
    }
    const std::pair<double, double> centroid{
        count > 0 ? sum_x / count : 0.0, count > 0 ? sum_y / count : 0.0};
    anchors.push_back(connector.NearestFree(centroid));
  }

  const bool have_start = goal.has_robot_pose();
  const std::pair<double, double> start =
      have_start
          ? connector.NearestFree(
                {goal.robot_pose().axis1(), goal.robot_pose().axis2()})
          : anchors.front();
  const double kUnreachable = std::numeric_limits<double>::max() / 4.0;
  std::vector<std::vector<double>> costs(
      rooms.size(), std::vector<double>(rooms.size(), kUnreachable));
  std::vector<double> from_start(rooms.size(), kUnreachable);
  for (std::size_t from = 0; from < rooms.size(); ++from) {
    CheckCanceled(cancel);
    for (std::size_t to = 0; to < rooms.size(); ++to) {
      if (from == to) {
        costs[from][to] = 0.0;
      } else {
        costs[from][to] = connector.Distance(anchors[from], anchors[to]);
      }
    }
    from_start[from] =
        have_start ? connector.Distance(start, anchors[from]) : (from == 0 ? 0.0
                                                                        : kUnreachable);
  }

  // Rooms are grouped by navigable connectivity. Only the group reachable
  // from the robot takes part in the tour; sealed rooms are still reported
  // with their own plan so the operator can see what is left behind a wall.
  std::vector<int> group(rooms.size(), -1);
  int groups = 0;
  for (std::size_t seed = 0; seed < rooms.size(); ++seed) {
    if (group[seed] >= 0) {
      continue;
    }
    std::vector<std::size_t> stack{seed};
    group[seed] = groups;
    while (!stack.empty()) {
      const std::size_t index = stack.back();
      stack.pop_back();
      for (std::size_t other = 0; other < rooms.size(); ++other) {
        if (group[other] < 0 && costs[index][other] < kUnreachable) {
          group[other] = groups;
          stack.push_back(other);
        }
      }
    }
    ++groups;
  }
  int active_group = group.front();
  if (have_start) {
    double best = kUnreachable;
    for (std::size_t index = 0; index < rooms.size(); ++index) {
      if (from_start[index] < best) {
        best = from_start[index];
        active_group = group[index];
      }
    }
  }

  std::vector<std::size_t> visit;
  if (active_group < 0) {
    for (std::size_t index = 0; index < rooms.size(); ++index) {
      visit.push_back(index);
    }
  } else {
    std::vector<std::size_t> active;
    std::vector<std::size_t> sealed;
    for (std::size_t index = 0; index < rooms.size(); ++index) {
      (group[index] == active_group ? active : sealed).push_back(index);
    }
    std::vector<std::vector<double>> local(
        active.size(), std::vector<double>(active.size(), 0.0));
    std::vector<double> local_start(active.size(), 0.0);
    for (std::size_t from = 0; from < active.size(); ++from) {
      for (std::size_t to = 0; to < active.size(); ++to) {
        local[from][to] = costs[active[from]][active[to]];
      }
      local_start[from] =
          have_start ? from_start[active[from]] : (active[from] == 0 ? 0.0
                                                                   : kUnreachable);
    }
    const auto ordered = RegionOrderer::Order(local, local_start);
    for (const std::size_t index : ordered.order) {
      visit.push_back(active[index]);
    }
    if (!sealed.empty()) {
      result.add_warnings(std::to_string(sealed.size()) + " of " +
                          std::to_string(rooms.size()) +
                          " room(s) are sealed off from the robot; open a door "
                          "to include them in the tour");
    }
    for (const std::size_t index : sealed) {
      visit.push_back(index);
    }
  }

  NavPath* nav = result.mutable_nav_path();
  nav->mutable_header()->set_frame_id(frame);
  auto* components = result.mutable_coverage_path();
  components->set_frame_id(frame);
  components->set_contains_turns(true);
  components->set_swaths_ordered(true);

  auto emit_pose = [&](double x, double y, double yaw) {
    auto* pose = nav->add_poses();
    pose->mutable_header()->set_frame_id(frame);
    auto* position = pose->mutable_pose()->mutable_position();
    position->set_x(x);
    position->set_y(y);
    auto* orientation = pose->mutable_pose()->mutable_orientation();
    orientation->set_z(std::sin(yaw * 0.5));
    orientation->set_w(std::cos(yaw * 0.5));
  };
  auto emit_leg = [&](const std::pair<double, double>& from,
                      const std::pair<double, double>& to) {
    const double distance =
        std::hypot(to.first - from.first, to.second - from.second);
    if (distance < 1e-6) {
      return;
    }
    const double yaw =
        std::atan2(to.second - from.second, to.first - from.first);
    const int steps = std::max(1, static_cast<int>(std::ceil(distance / 0.15)));
    emit_pose(from.first, from.second, yaw);
    for (int step = 1; step < steps; ++step) {
      const double ratio = static_cast<double>(step) / steps;
      emit_pose(from.first + (to.first - from.first) * ratio,
                from.second + (to.second - from.second) * ratio, yaw);
    }
  };

  double weighted_coverage = 0.0;
  double total_area = 0.0;
  double travel = 0.0;
  double task_time = 0.0;
  std::pair<double, double> cursor = start;
  bool have_cursor = have_start;
  int index = 0;
  int connected = 0;
  int planned = 0;
  for (const std::size_t room_index : visit) {
    CheckCanceled(cancel);
    const bool reachable = active_group >= 0 && group[room_index] == active_group;
    stage("room " + std::to_string(index + 1) + "/" +
              std::to_string(visit.size()),
          0.15 + 0.75 * static_cast<double>(index) / visit.size());
    const F2CCells& cells = rooms[room_index];
    auto* region = result.add_regions();
    region->set_id(index);
    region->set_order(index + 1);
    region->set_name("Region " + std::to_string(index + 1));
    region->set_color_index(index % 8);
    const double area = RegionSegmenter::Area(cells);
    region->set_area(area);
    for (const auto& cell : cells) {
      for (const auto& ring : cell) {
        const auto points = CleanRing(ring);
        if (points.size() < 3) {
          continue;
        }
        auto* polygon = region->add_polygons();
        for (const auto& point : points) {
          auto* coordinate = polygon->add_coordinates();
          coordinate->set_axis1(point.first);
          coordinate->set_axis2(point.second);
        }
      }
    }

    RoomCoverage room;
    try {
      room = PlanRoom(cells, config, goal, headland);
    } catch (const std::exception& error) {
      result.add_warnings("Region " + std::to_string(index + 1) + ": " +
                          error.what());
      // A headland band that cannot host a turn still leaves the room
      // coverable: retry with a plain boustrophedon sweep.
      try {
        room = PlanRoom(cells, config, goal, false);
      } catch (const std::exception& retry) {
        // Household rooms are tight: fall back to a turning radius suited to
        // the robot footprint before reporting the room as unplannable.
        Configuration tight = config;
        tight.min_turning_radius =
            std::max(0.15, std::min(config.min_turning_radius,
                                    radius > 0.0 ? radius : 0.4));
        try {
          room = PlanRoom(cells, tight, goal, false);
        } catch (const std::exception& last) {
          result.add_warnings("Region " + std::to_string(index + 1) +
                              " tight-radius retry failed: " + last.what());
        }
        result.add_warnings("Region " + std::to_string(index + 1) +
                            " retry failed: " + retry.what());
      }
    }
    ComputeAction::Result planned_room;
    F2CField room_field(cells);
    room_field.SetRefPoint(F2CPoint(0.0, 0.0));
    room_field.SetCRS(frame);
    if (room.swaths.size() > 0) {
      ExportSwaths(room.swaths, room_field, frame, true, true,
                   *planned_room.mutable_coverage_path());
    }
    if (room.path.size() > 1) {
      ExportPath(room.path, room_field, frame, true,
                 config.turn_point_distance, planned_room);
    }
    for (const auto& swath : planned_room.coverage_path().swaths()) {
      *components->add_swaths() = swath;
    }
    for (const auto& state : planned_room.coverage_path().states()) {
      *components->add_states() = state;
    }
    *region->mutable_nav_path() = planned_room.nav_path();
    for (const auto& state : planned_room.coverage_path().states()) {
      *region->add_states() = state;
    }
    region->set_coverage(reachable ? room.coverage : 0.0);
    region->set_swath_count(reachable ? room.swath_count : 0);
    if (reachable && planned_room.nav_path().poses_size() > 0) {
      const auto& entry_pose = planned_room.nav_path().poses(0).pose().position();
      const std::pair<double, double> entry{entry_pose.x(), entry_pose.y()};
      if (have_cursor) {
        const auto link = connector.Connect(cursor, entry);
        if (!link.points.empty()) {
          for (std::size_t point = 1; point < link.points.size(); ++point) {
            emit_leg(link.points[point - 1], link.points[point]);
          }
          travel += link.distance;
          ++connected;
        }
      }
      for (const auto& pose : planned_room.nav_path().poses()) {
        *nav->add_poses() = pose;
      }
      const auto& last = nav->poses(nav->poses_size() - 1).pose().position();
      cursor = {last.x(), last.y()};
      have_cursor = true;
      ++planned;
    }
    if (area > 1e-9) {
      weighted_coverage += (reachable ? room.coverage : 0.0) * area;
    }
    total_area += area;
    task_time += room.path.GetTaskTime();
    ++index;
  }
  result.set_core_coverage(total_area > 0.0 ? weighted_coverage / total_area
                                            : 0.0);
  result.set_field_wkt((field + master.GetRefPoint()).ExportToWkt());
  result.set_core_wkt(result.field_wkt());
  result.set_travel_distance(travel);
  result.set_task_time(std::isfinite(task_time) ? task_time : 0.0);
  result.add_warnings(std::to_string(planned) + " room(s) planned through " +
                      std::to_string(connected) +
                      " traversable connection(s)");
  return result;
}

}  // namespace
CoverageServer::CoverageServer(Configuration c) : config_(std::move(c)) {
  RobotParams validate(config_);
}
void ExportSwaths(const F2CSwaths& swaths, const F2CField& field,
                  const std::string& frame, bool cartesian, bool ordered,
                  proto::PathComponents& output) {
  output.set_frame_id(frame);
  output.set_swaths_ordered(ordered);
  for (const auto& swath : swaths) {
    auto* msg = output.add_swaths();
    msg->set_id(swath.GetId());
    msg->set_width(swath.GetWidth());
    msg->mutable_path()->mutable_header()->set_frame_id(frame);
    for (const auto& p : swath.GetPath())
      Pose(*msg->mutable_path(), OutputPoint(p, field, cartesian),
           swath.GetInAngle(), frame);
  }
}
void ExportPath(const F2CPath& path, const F2CField& field,
                const std::string& frame, bool cartesian, double step,
                ComputeAction::Result& output) {
  auto* components = output.mutable_coverage_path();
  components->set_frame_id(frame);
  components->set_contains_turns(true);
  components->set_swaths_ordered(true);
  auto* nav = output.mutable_nav_path();
  nav->mutable_header()->set_frame_id(frame);
  NavPath* current_turn = nullptr;
  for (const auto& state : path) {
    const auto p = OutputPoint(state.point, field, cartesian);
    auto* msg = components->add_states();
    msg->set_x(p.GetX());
    msg->set_y(p.GetY());
    msg->set_z(p.GetZ());
    msg->set_angle(state.angle);
    msg->set_len(state.len);
    msg->set_direction(static_cast<int>(state.dir));
    msg->set_type(static_cast<int>(state.type));
    msg->set_velocity(state.velocity);
    if (state.type != f2c::types::PathSectionType::SWATH) {
      if (!current_turn) {
        current_turn = components->add_turns();
        current_turn->mutable_header()->set_frame_id(frame);
      }
      Pose(*current_turn, p, state.angle, frame);
    } else
      current_turn = nullptr;
    const unsigned count =
        std::max(1u, static_cast<unsigned>(std::ceil(state.len / step)));
    if (nav->poses_size() + count > 1000000)
      throw RequestException("Coverage path exceeds one million sampled poses");
    for (unsigned i = 0; i < count; ++i) {
      const auto sample = state.point.GetPointFromAngle(
          state.angle, state.len * static_cast<int>(state.dir) * i / count);
      Pose(*nav, OutputPoint(sample, field, cartesian), state.angle, frame);
      components->add_velocities(state.velocity);
      components->add_is_backward(state.dir ==
                                  f2c::types::PathDirection::BACKWARD);
    }
  }
  if (path.size()) {
    Pose(*nav, OutputPoint(path.AtEnd(), field, cartesian), path.back().angle,
         frame);
    components->add_velocities(0);
    components->add_is_backward(path.back().dir ==
                                f2c::types::PathDirection::BACKWARD);
  }
}
ComputeAction::Result CoverageServer::Compute(
    const proto::CoverageRequest& goal, const CancelChecker& cancel,
    const Progress& feedback) const {
  const auto began = std::chrono::steady_clock::now();
  ComputeAction::Result result;
  auto stage = [&](const std::string& name, double progress) {
    CheckCanceled(cancel);
    if (feedback) feedback(name, progress);
  };
  try {
    Configuration effective = config_;
    if (goal.has_robot_radius()) {
      if (!std::isfinite(goal.robot_radius()) || goal.robot_radius() <= 0.0)
        throw RequestException("robot_radius must be a positive metre value");
      effective.robot_radius = goal.robot_radius();
      effective.operation_width = 2.0 * goal.robot_radius();
      RobotParams validate(effective);
    }
    if (goal.has_swath_spacing()) {
      if (!std::isfinite(goal.swath_spacing()) ||
          goal.swath_spacing() <= 0.0) {
        throw RequestException(
            "swath_spacing must be a positive metre value");
      }
      effective.operation_width = goal.swath_spacing();
      RobotParams validate(effective);
    }
    const bool headland =
        Enabled(goal.has_generate_headland(), goal.generate_headland());
    const bool route_enabled =
        Enabled(goal.has_generate_route(), goal.generate_route());
    const bool path_enabled =
        Enabled(goal.has_generate_path(), goal.generate_path());
    const bool cartesian = goal.has_coordinates_in_cartesian_frame()
                               ? goal.coordinates_in_cartesian_frame()
                               : effective.cartesian;
    if (path_enabled && !route_enabled)
      throw RequestException("generate_path requires generate_route");
    if (goal.generate_headland_swaths() && (!headland || !path_enabled))
      throw RequestException(
          "Headland swaths require the complete headland/route/path pipeline");
    if (goal.rows().empty() && !Unknown(goal.row_swath_mode().mode()))
      throw RequestException("Row mode requires annotated rows");
    if (goal.use_start_pose() &&
        (!route_enabled ||
         Upper(RouteGenerator(effective).Resolve(goal.route_mode()).mode()) !=
             "TSP"))
      throw RequestException("start_pose requires TSP route mode");
    stage("field", 0.0);
    F2CField master;
    std::string frame = goal.frame_id().empty() ? "map" : goal.frame_id();
    if (goal.use_gml_file()) {
      if (goal.gml_field().empty())
        throw RequestException("Missing GML filename");
      master = f2c::Parser::ImportFieldGml(goal.gml_field(), true);
      if (!master.GetCRS().empty()) frame = master.GetCRS();
    } else {
      if (goal.polygons().empty())
        throw RequestException("Missing field polygon");
      F2CCell cell;
      for (const auto& polygon : goal.polygons()) {
        if (polygon.coordinates_size() < 3)
          throw RequestException("Ring needs at least three vertices");
        F2CLinearRing ring;
        for (const auto& p : polygon.coordinates()) {
          if (!std::isfinite(p.axis1()) || !std::isfinite(p.axis2()))
            throw RequestException("Nonfinite coordinate");
          ring.AddPoint(p.axis1(), p.axis2());
        }
        ring.CloseRing();
        cell.AddRing(ring);
      }
      Validate(F2CCells(cell));
      master = F2CField(F2CCells(cell));
      master.SetCRS(frame);
    }
    if (!cartesian) {
      if (master.GetCRS() != "EPSG:4326" && master.GetCRS() != "EPSG:4258")
        throw RequestException("GPS requests require EPSG:4326 or EPSG:4258");
      f2c::Transform::TransformToUTM(master);
    }
    auto field = master.GetField().Clone();
    Validate(field);
    // A map-driven request plans every room on its own: one global plan over
    // the bounding rectangle has to cross walls and its turn connections are
    // frequently infeasible. Per-room plans are stitched afterwards.
    if (goal.generate_regions() && goal.has_map()) {
      result = PlanMapRegions(effective, goal, master, field, frame, cancel,
                              stage);
      stage("complete", 1.0);
      result.set_error_code(proto::NONE);
      result.set_planning_time(
          std::chrono::duration<double>(std::chrono::steady_clock::now() - began)
              .count());
      return result;
    }
    stage("headland", 0.1);
    HeadlandGenerator hg(effective);
    auto core = headland ? hg.GenerateHeadlands(field, goal.headland_mode())
                         : field.Clone();
    F2CCells travel = core.Clone(), swath_cells = core.Clone();
    if (goal.generate_decomp()) {
      stage("decomposition", 0.2);
      travel = DecompGenerator(effective).Decompose(core, goal.decomp_mode());
      swath_cells = headland ? hg.GenerateHeadlandsBetweenCells(
                                   travel, goal.route_headland_width() > 0
                                               ? goal.route_headland_width()
                                               : effective.operation_width)
                             : travel.Clone();
    }
    Validate(swath_cells);
    stage("swaths", 0.3);
    F2CSwathsByCells grouped;
    if (!goal.rows().empty()) {
      if (goal.generate_decomp() || !cartesian)
        throw RequestException(
            "Annotated row input currently requires one Cartesian field");
      auto rows = GenerateRowSwaths(goal, master.GetRefPoint());
      F2CSwaths clipped;
      for (const auto& row : rows) {
        for (const auto& line : field.GetLinesInside(row.GetPath()))
          if (line.size() >= 2 && line.Length() > 1e-8)
            clipped.emplace_back(line.Clone(), row.GetWidth(), row.GetId());
      }
      grouped.emplace_back(clipped);
      swath_cells = field.Clone();
      travel = field.Clone();
      core = field.Clone();
    } else
      grouped = SwathGenerator(effective).GenerateSwathsByCells(
          swath_cells, goal.swath_mode());
    if (grouped.SizeTotal() == 0 || grouped.SizeTotal() > 1000)
      throw RequestException("Coverage needs 1..1000 swaths");
    F2CSwaths swaths;
    for (const auto& group : grouped) swaths.Append(group);
    F2CPath path;
    F2CRoute route;
    if (route_enabled) {
      stage("route", 0.55);
      std::optional<F2CPoint> start;
      if (goal.use_start_pose()) {
        F2CPoint p(goal.start_pose().axis1(), goal.start_pose().axis2());
        if (!cartesian)
          p = f2c::Transform::transform(p, master.GetPrevCRS(),
                                        master.GetCRS());
        start = p - master.GetRefPoint();
        if (!field.IsPointIn(*start))
          throw RequestException("start_pose must lie in traversable field");
      }
      route = RouteGenerator(effective).GenerateRoute(
          travel, swath_cells, grouped, goal.route_mode(), start);
      route.SetField(field);
      swaths = Flatten(route);
      if (swaths.size() != grouped.SizeTotal())
        throw std::runtime_error("Route lost generated swaths");
      if (path_enabled) {
        stage("path", 0.7);
        PathGenerator pp(effective);
        path = pp.GeneratePath(route, goal.path_mode());
        if (goal.generate_headland_swaths()) {
          stage("headland_swaths", 0.8);
          F2CPath perimeter;
          if (goal.headland_full_coverage()) {
            for (const auto& ring : hg.GenerateHeadlandSwaths(
                     field, effective.operation_width, goal.headland_mode())) {
              auto next = pp.GeneratePerimeter(ring, field, path.AtStart(), goal.path_mode());
              if (perimeter.size() > 0 && next.size() > 0) {
                RobotParams ring_robot(effective);
                auto ring_turn = pp.CreateCurve(goal.path_mode());
                perimeter += f2c::pp::PathPlanning::PlanPathForConnection(
                    ring_robot.GetRobot(), perimeter.AtEnd(), perimeter.back().angle,
                    F2CMultiPoint(), next.AtStart(), next[0].angle, *ring_turn, &field);
              }
              perimeter += next;
            }
          } else
            perimeter = pp.GeneratePerimeter(core, field, path.AtStart(),
                                             goal.path_mode());
          RobotParams params(effective);
          auto turn = pp.CreateCurve(goal.path_mode());
          auto connection = [&](const F2CPath& a, const F2CPath& b) {
            return f2c::pp::PathPlanning::PlanPathForConnection(
                params.GetRobot(), a.AtEnd(), a.back().angle, F2CMultiPoint(),
                b.AtStart(), b[0].angle, *turn, &field);
          };
          F2CPath combined;
          if (goal.headland_first()) {
            combined = perimeter;
            combined += connection(perimeter, path);
            combined += path;
          } else {
            combined = path;
            combined += connection(path, perimeter);
            combined += perimeter;
          }
          path = std::move(combined);
        }
        stage("validation", 0.9);
        RobotParams robot(effective);
        f2c::pp::detail::FreeSpace free(field);
        if (!f2c::pp::detail::ValidPath(path, robot.GetRobot(), &free)) {
          for (size_t i = 0; i < path.size(); ++i) {
            F2CPath pair;
            pair.AddState(path[i]);
            if (i + 1 < path.size()) pair.AddState(path[i + 1]);
            if (!f2c::pp::detail::ValidPath(pair, robot.GetRobot(), &free))
              throw std::runtime_error(
                  "Final path constraint failure at state " +
                  std::to_string(i) + ": " + pair.SerializePath(9));
          }
          throw std::runtime_error(
              "Final path violates field, hole, join or curvature constraints");
        }
        for (const auto& state : path)
          if (!std::isfinite(state.velocity) || state.velocity < 0)
            throw std::runtime_error("Invalid path velocity");
        ExportPath(path, master, frame, cartesian, effective.turn_point_distance,
                   result);
        const double task_time = path.GetTaskTime();
        result.set_task_time(std::isfinite(task_time) ? task_time : 0);
      }
    }
    ExportSwaths(swaths, master, frame, cartesian, route_enabled,
                 *result.mutable_coverage_path());
    result.set_core_coverage(
        f2c::obj::FieldCoverage().ComputeCost(core, swaths));
    result.set_field_wkt((field + master.GetRefPoint()).ExportToWkt());
    result.set_core_wkt((core + master.GetRefPoint()).ExportToWkt());
    if (!cartesian)
      result.add_warnings(
          "GPS output poses use geographic units; execution requires a "
          "Cartesian request");
    if (goal.generate_regions()) {
      const F2CPoint reference = master.GetRefPoint();
      const bool map_based = goal.has_map();
      RegionSegmenter::Options options;
      options.columns = goal.region_columns() > 0 ? goal.region_columns() : 2;
      options.rows = goal.region_rows() > 0 ? goal.region_rows() : 2;
      std::vector<F2CCells> regions =
          map_based ? RegionSegmenter::MapSplit(goal.map(), RegionSegmenter::MapOptions{})
                    : RegionSegmenter::GridSplit(field, options);
      std::vector<std::pair<double, double>> centroids;
      for (const auto& cells : regions) {
        if (cells.IsEmpty()) {
          centroids.emplace_back(0.0, 0.0);
          continue;
        }
        double sum_x = 0.0;
        double sum_y = 0.0;
        int count = 0;
        for (const auto& point : cells.GetCellBorder(0)) {
          sum_x += point.GetX() + (map_based ? 0.0 : reference.GetX());
          sum_y += point.GetY() + (map_based ? 0.0 : reference.GetY());
          ++count;
        }
        centroids.emplace_back(count > 0 ? sum_x / count : 0.0,
                               count > 0 ? sum_y / count : 0.0);
      }
      const double start_x = goal.use_start_pose() ? goal.start_pose().axis1() : 0.0;
      const double start_y = goal.use_start_pose() ? goal.start_pose().axis2() : 0.0;
      const auto ordered = RegionOrderer::Order(centroids, start_x, start_y);
      std::vector<F2CCells> ordered_regions;
      ordered_regions.reserve(regions.size());
      for (const std::size_t index : ordered.order) {
        ordered_regions.push_back(std::move(regions[index]));
      }
      regions = std::move(ordered_regions);
      result.set_travel_distance(ordered.distance);
      const auto shift_cells = [](const F2CCells& cells, double dx,
                                  double dy) {
        F2CCells output;
        for (const auto& cell : cells) {
          F2CCell shifted;
          for (const auto& ring : cell) {
            const auto points = CleanRing(ring);
            if (points.size() < 3) {
              continue;
            }
            F2CLinearRing result_ring;
            for (const auto& point : points) {
              result_ring.AddPoint(point.first + dx, point.second + dy);
            }
            result_ring.CloseRing();
            shifted.AddRing(result_ring);
          }
          if (shifted.Area() > 1e-6) {
            output.AddGeometry(shifted);
          }
        }
        return output;
      };
      int region_index = 0;
      for (const auto& cells : regions) {
        const F2CCells local =
            shift_cells(cells, map_based ? -reference.GetX() : 0.0,
                        map_based ? -reference.GetY() : 0.0);
        const F2CCells display =
            shift_cells(cells, map_based ? 0.0 : reference.GetX(),
                        map_based ? 0.0 : reference.GetY());
        if (local.IsEmpty() || display.IsEmpty()) {
          continue;
        }
        auto* region = result.add_regions();
        region->set_id(region_index);
        region->set_order(region_index + 1);
        region->set_name("Region " + std::to_string(region_index + 1));
        region->set_color_index(region_index % 8);
        const double area = RegionSegmenter::Area(display);
        region->set_area(area);
        for (const auto& region_cell : display) {
          for (const auto& ring : region_cell) {
            const auto points = CleanRing(ring);
            if (points.size() < 3) {
              continue;
            }
            auto* polygon = region->add_polygons();
            for (const auto& point : points) {
              auto* coordinate = polygon->add_coordinates();
              coordinate->set_axis1(point.first);
              coordinate->set_axis2(point.second);
            }
          }
        }
        region->set_coverage(
            f2c::obj::FieldCoverage().ComputeCost(local, swaths));
        int swath_count = 0;
        for (const auto& swath : swaths) {
          if (local.IsPointIn(swath.GetPath().StartPoint())) {
            ++swath_count;
          }
        }
        region->set_swath_count(swath_count);
        for (const auto& state : result.coverage_path().states()) {
          if (local.IsPointIn(F2CPoint(state.x(), state.y()))) {
            *region->add_states() = state;
          }
        }
        ++region_index;
      }
    }
    stage("complete", 1.0);
    result.set_error_code(proto::NONE);
  } catch (const CanceledException& e) {
    result.Clear();
    result.set_error_code(proto::CANCELED);
    result.set_error_msg(e.what());
  } catch (const CoverageException& e) {
    result.Clear();
    result.set_error_code(proto::INVALID_MODE_SET);
    result.set_error_msg(e.what());
  } catch (const std::invalid_argument& e) {
    result.Clear();
    result.set_error_code(proto::INVALID_REQUEST);
    result.set_error_msg(e.what());
  } catch (const std::exception& e) {
    result.Clear();
    result.set_error_code(proto::INTERNAL_F2C_ERROR);
    result.set_error_msg(e.what());
  }
  result.set_planning_time(
      std::chrono::duration<double>(std::chrono::steady_clock::now() - began)
          .count());
  return result;
}
}  // namespace autonomy::task::coverage
