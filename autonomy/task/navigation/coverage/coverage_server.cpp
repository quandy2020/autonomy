// Copyright (c) 2023 Open Navigation LLC; Autonomy adaptation 2026.
// SPDX-License-Identifier: Apache-2.0
/** @file coverage_server.cpp
 * @brief Native complete-coverage coverage server implementation. */
#include "autonomy/task/navigation/coverage/coverage_server.hpp"

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
      const std::vector<F2CCells> regions =
          map_based ? RegionSegmenter::MapSplit(goal.map(), RegionSegmenter::MapOptions{})
                    : RegionSegmenter::GridSplit(field, options);
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
