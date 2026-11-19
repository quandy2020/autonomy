/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file path_planning.cpp
 * @brief Fields2Cover path planning implementation.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */



#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>
#include "autonomy/planning/planner/fields2cover/path_planning/constrained_connection.hpp"
#include "autonomy/planning/planner/fields2cover/path_planning/path_planning.hpp"

namespace autonomy::planning::planner::fields2cover::pp {

namespace {

// Deflection at each corner of `poly`. The ends take the swath heading where
// there is one: the track's own first/last leg is often a spur off the border
// graph the robot never drives (see PR-4, B.3).
std::vector<double> CornerSweeps(const std::vector<F2CPoint>& poly,
    const std::optional<double>& start_angle,
    const std::optional<double>& end_angle) {
  const size_t n = poly.size();
  const auto LegAngle = [&poly](size_t a, size_t b) {
      return (poly[b] - poly[a]).GetAngleFromPoint();
    };
  std::vector<double> sweeps(n, 0.0);
  for (size_t k = 1; k + 1 < n; ++k) {
    sweeps[k] = F2CPoint::GetAngleDiffAbs(LegAngle(k - 1, k), LegAngle(k, k + 1));
  }
  if (start_angle && n > 1) {
    sweeps.front() = F2CPoint::GetAngleDiffAbs(*start_angle, LegAngle(0, 1));
  }
  if (end_angle && n > 1) {
    sweeps.back() = F2CPoint::GetAngleDiffAbs(LegAngle(n - 2, n - 1), *end_angle);
  }
  return sweeps;
}

// Corners from `i` on that sit too close together to round one at a time:
// rounding the first would leave the next no approach.
size_t FoldedSpan(const std::vector<F2CPoint>& poly,
    const std::vector<double>& sweeps, size_t i, const F2CRobot& robot) {
  const double gap = 2.0 * robot.GetRadiusMargin() * robot.GetMinTurningRadius();
  size_t span = 1;
  while (i + span < poly.size() && span < robot.GetMaxCornerSpan() &&
      sweeps[i + span] >= robot.GetMinSweep() &&
      poly[i + span - 1].Distance(poly[i + span]) < gap) {
    ++span;
  }
  return span;
}

// Where a maneuver through poly[i..j] may start and end, and what it must turn.
struct Span {
  F2CPoint first, last_corner, after;
  double in_angle, out_angle, net, total;
  double lo, in_reach, out_reach;
  bool pin_in, pin_out, uturn;
};

// Size the maneuver through poly[i..j]. Unset if no turn fits between the
// room the legs leave and the ground `cut_tol` lets it cut.
std::optional<Span> SpanGeometry(const std::vector<F2CPoint>& poly,
    const std::vector<double>& sweeps, size_t i, size_t j,
    const F2CPoint& cursor, const F2CRobot& robot, double cut_tol,
    bool continuous, const std::optional<double>& start_angle,
    const std::optional<double>& end_angle) {
  const size_t n = poly.size();
  const double radius = robot.GetMinTurningRadius();
  Span s;
  // At either end the pose is pinned to the swath, not slid along a leg.
  s.pin_in = (i == 0);
  s.pin_out = (j + 1 == n);
  s.first = poly[i];
  s.last_corner = poly[j];
  s.after = s.pin_out ? poly[j] : poly[j + 1];

  // Total turning sizes the maneuver's Length; net deflection (which can be
  // smaller, e.g. on an S where corners cancel) sizes its geometry.
  s.total = 0.0;
  for (size_t k = i; k <= j; ++k) {
    s.total += sweeps[k];
  }
  s.in_angle = s.pin_in ?
      *start_angle : (s.first - cursor).GetAngleFromPoint();
  s.out_angle = s.pin_out ?
      *end_angle : (s.after - s.last_corner).GetAngleFromPoint();
  s.net = F2CPoint::GetAngleDiffAbs(s.in_angle, s.out_angle);

  const bool tail = (j + 2 == n) && sweeps[n - 1] < robot.GetMinSweep();
  const double back_room = s.pin_in ? 0.0 : cursor.Distance(s.first);
  const double fwd_room = s.pin_out ? 0.0 :
      (tail ? 1.0 : robot.GetLegShare()) * s.last_corner.Distance(s.after);
  const double room = s.pin_in ? fwd_room : (s.pin_out ? back_room :
      std::min(back_room, fwd_room));

  // A short reversal is a u-turn (swings past the vertices into the headland),
  // not a corner to fillet; a long one is a detour to follow.
  const double hop = s.first.Distance(s.last_corner);
  s.uturn = s.net > robot.GetReversalSweep() && hop < robot.GetUturnMaxHop();

  // Offset of the outgoing leg from the entry heading's line.
  const double sep = std::fabs(
      std::cos(s.in_angle) * (s.after.GetY() - s.first.GetY()) -
      std::sin(s.in_angle) * (s.after.GetX() - s.first.GetX()));

  const double turn_radius = robot.GetTurnRadius(s.net, continuous);
  s.lo = robot.GetMinBackoffRadii() * turn_radius;
  double hi = room;
  if (!s.uturn) {
    if (s.net > M_PI - 1e-3) {
      return {};  // reversal too wide to be anything but a detour
    }
    const double tangent_margin = continuous ? robot.GetRadiusMargin() : 1.0;
    s.lo = std::max(tangent_margin * turn_radius * std::tan(0.5 * s.net), s.lo);
    hi = std::min(hi, cut_tol / std::max(std::tan(0.25 * s.net), 1e-6));
  }
  // On a jog net cancels to ~0 and the tan(net/4) bound with it; fall back to
  // bounding the offset by the ground the maneuver makes up. At a pinned end
  // taking a single corner there is no outgoing leg to measure that against --
  // `after` is the corner itself -- so the bound would collapse to zero.
  if (!s.pin_out || j > i) {
    hi = std::min(hi, sep + robot.GetRadiusMargin() * (1.0 + s.total) * radius);
  }
  if (s.lo > hi) {
    return {};
  }

  // A fillet meets both legs the same Distance out; a u-turn instead takes
  // whatever room each leg has, to turn a cusp into a drivable arc.
  s.in_reach = hi;
  s.out_reach = hi;
  if (s.uturn) {
    const double reach = robot.GetUturnReachRadii() * turn_radius;
    s.in_reach = std::max(s.lo, std::min(back_room, reach));
    s.out_reach = std::max(s.lo, std::min(fwd_room, reach));
  }
  if (s.pin_in) {
    s.in_reach = 0.0;
  }
  if (s.pin_out) {
    s.out_reach = 0.0;
  }
  return s;
}

// A turn accepted for a span, and the points it runs between.
struct Turn {
  F2CPath arc;
  F2CPoint entry, exit;
};

// Widen the offset until the planner returns a turn that rounds the span
// rather than looping around it. Unset if none of them does.
std::optional<Turn> PlanSpanTurn(const Span& s,
    const std::vector<F2CPoint>& poly, size_t i, size_t j,
    const F2CPoint& cursor, const F2CRobot& robot, TurningBase& turn,
    double cut_tol, bool continuous) {
  // A fillet is tried tightest first (it cuts least), a u-turn widest. Sampled
  // finely because the planner can refuse one specific offset for no geometric
  // reason, and jumping to a much wider one would cut more than needed.
  const std::array<double, 6> fracs = s.uturn ?
      std::array<double, 6>{1.0, 0.75, 0.5, 0.25, 0.1, 0.0} :
      std::array<double, 6>{0.0, 0.1, 0.25, 0.5, 0.75, 1.0};

  for (const double frac : fracs) {
    Turn t;
    const double back_off = s.pin_in ? 0.0 : s.lo + frac * (s.in_reach - s.lo);
    const double fwd_off = s.pin_out ? 0.0 : s.lo + frac * (s.out_reach - s.lo);
    t.entry = s.first.GetPointAlong(cursor, back_off);
    t.exit = s.last_corner.GetPointAlong(s.after, fwd_off);
    t.arc = turn.CreateTurn(robot, t.entry, s.in_angle, t.exit, s.out_angle);
    if (t.arc.size() == 0) {
      continue;
    }

    // A feasible pose pair can still come back as a loop, or as an S that
    // doubles back before Settling. Both turn far more than the span asks for,
    // which Length measures only indirectly -- a loop's extra 2*pi is cheap at
    // a small radius. Compare the turning itself.
    double turned = 0.0;
    for (size_t k = 1; k < t.arc.size(); ++k) {
      turned += F2CPoint::GetAngleDiffAbs(t.arc[k].angle, t.arc[k - 1].angle);
    }
    // A u-turn earns its teardrop; anything else should turn once.
    if (turned > s.total + (s.uturn ? M_PI : robot.GetTurnSlack())) {
      continue;
    }

    std::vector<F2CPoint> track{t.entry};
    for (size_t k = i; k <= j; ++k) {
      track.push_back(poly[k]);
    }
    track.push_back(t.exit);
    // A u-turn is allowed the corridor plus the diameter it swings past.
    const double dev_max = s.uturn ?
        cut_tol + 2.0 * robot.GetTurnRadius(s.net, continuous) : cut_tol;
    if (t.arc.MaxDistanceTo(F2CLineString(track)) > dev_max) {
      continue;
    }
    return t;
  }
  return {};
}

// Follow `poly`: straight runs as they are, corners through the turn planner.
// A corner only a turn cutting past `cut_tol` could round is left square.
void AppendRoundedTrack(
    F2CPath& path, const std::vector<F2CPoint>& poly, const F2CRobot& robot,
    TurningBase& turn, double cut_tol, bool continuous,
    const std::optional<double>& start_angle, const std::optional<double>& end_angle) {
  const size_t n = poly.size();
  const std::vector<double> sweeps = CornerSweeps(poly, start_angle, end_angle);

  F2CPoint cursor = poly.front();
  size_t i = 0;
  while (i < n) {
    // A short first leg may be a border spur. Fold it into the next turn
    // while the entry pose is still pinned; driving it first leaves too
    // little approach to satisfy the minimum turning radius.
    const bool short_entry = i == 0 && start_angle && n > 2 &&
        sweeps[1] >= robot.GetMinSweep() &&
        poly[0].Distance(poly[1]) <
            2.0 * robot.GetRadiusMargin() * robot.GetMinTurningRadius();
    if (sweeps[i] < robot.GetMinSweep() && !short_entry) {
      path.AppendStraight(cursor, poly[i], robot.GetCruiseVel(),
          autonomy::planning::planner::fields2cover::types::PathSectionType::HL_SWATH);
      cursor = poly[i];
      ++i;
      continue;
    }

    std::optional<Turn> rounded;
    size_t last = i;
    // Widening out from the fold, because a shorter span cuts less corner.
    for (size_t span = FoldedSpan(poly, sweeps, i, robot);
        span <= robot.GetMaxCornerSpan() && i + span <= n && !rounded; ++span) {
      const size_t j = i + span - 1;
      if (j + 1 == n && !end_angle) {
        break;  // an open route end holds no heading, so it is no corner
      }
      const std::optional<Span> s = SpanGeometry(poly, sweeps, i, j, cursor,
          robot, cut_tol, continuous, start_angle, end_angle);
      if (!s) {
        continue;
      }
      rounded = PlanSpanTurn(*s, poly, i, j, cursor, robot, turn,
          cut_tol, continuous);
      last = j;
    }

    if (rounded) {
      path.AppendStraight(cursor, rounded->entry, robot.GetCruiseVel(),
          autonomy::planning::planner::fields2cover::types::PathSectionType::HL_SWATH);
      path += rounded->arc;  // as returned, Reverse legs included
      cursor = rounded->exit;
      i = last + 1;
    } else {
      path.AppendStraight(cursor, poly[i], robot.GetCruiseVel(),
          autonomy::planning::planner::fields2cover::types::PathSectionType::HL_SWATH);
      cursor = poly[i];
      ++i;
    }
  }
  path.AppendStraight(cursor, poly.back(), robot.GetCruiseVel(),
      autonomy::planning::planner::fields2cover::types::PathSectionType::HL_SWATH);
}

}  // namespace

F2CPath PathPlanning::PlanPath(const F2CRobot& robot,
    const F2CRoute& route, TurningBase& turn) {
  F2CPath path;
  const F2CCells* field = route.GetField() ? &*route.GetField() : nullptr;
  for (size_t i = 0; i < route.SizeVectorSwaths(); ++i) {
    auto prev_swaths = (i >0) ? route.GetSwaths(i-1) : F2CSwaths();
    path += PlanPathForConnection(robot,
        prev_swaths, route.GetConnection(i), route.GetSwaths(i), turn, field);
    const auto& group = route.GetSwaths(i);
    for (size_t j = 0; j < group.size(); ++j) {
      if (j > 0) {
        path += PlanPathForConnection(robot, group[j - 1].EndPoint(),
            group[j - 1].GetOutAngle(), F2CMultiPoint(), group[j].StartPoint(),
            group[j].GetInAngle(), turn, field);
      }
      path.AppendSwath(group[j], robot.GetCruiseVel());
    }
  }
  if (route.SizeConnections() > route.SizeVectorSwaths()) {
    path += PlanPathForConnection(robot,
      route.GetLastSwaths(), route.GetLastConnection(), F2CSwaths(), turn, field);
  }
  return path;
}

F2CPath PathPlanning::PlanPath(const F2CRobot& robot,
    const F2CSwaths& swaths, TurningBase& turn) {
  F2CPath path;
  if (swaths.size() > 1) {
    for (size_t i = 0; i < swaths.size()-1; ++i) {
      path.AppendSwath(swaths[i], robot.GetCruiseVel());
      F2CPath turn_path = turn.CreateTurn(robot,
          swaths[i].EndPoint(), swaths[i].GetOutAngle(),
          swaths[i + 1].StartPoint(), swaths[i + 1].GetInAngle());
      path += turn_path.discretize(turn.GetDiscretization());
    }
  }
  if (swaths.size() > 0) {
    path.AppendSwath(swaths.back(), robot.GetCruiseVel());
  }
  return path;
}

F2CPath PathPlanning::PlanPathForConnection(const F2CRobot& robot,
    const F2CSwaths& s1,
    const F2CMultiPoint& mp,
    const F2CSwaths& s2,
    TurningBase& turn, const F2CCells* field) {
  F2CPoint p1, p2;
  double ang1, ang2;

  if (s1.size() > 0) {
    p1 = s1.back().EndPoint();
    ang1 = s1.back().GetOutAngle();
  } else if (mp.size() > 0) {
    p1 = mp[0];
    ang1 = mp.GetOutAngle(0);
  } else {
    return {};
  }
  if (s2.size() > 0) {
    p2 = s2[0].StartPoint();
    ang2 = s2[0].GetInAngle();
  } else if (mp.size() > 0) {
    p2 = mp.GetLastPoint();
    ang2 = mp.GetInAngle(mp.size()-1);
  } else {
    return {};
  }
  return PlanPathForConnection(robot, p1, ang1, mp, p2, ang2, turn, field);
}

F2CPath PathPlanning::PlanPathForConnection(const F2CRobot& robot,
    const F2CPoint& p1, double ang1,
    const F2CMultiPoint& mp,
    const F2CPoint& p2, double ang2,
    TurningBase& turn, const F2CCells* field) {
  std::vector<F2CPoint> pts{p1};
  const std::vector<F2CPoint> mid = mp.ToVectorPoint();
  pts.insert(pts.end(), mid.begin(), mid.end());
  pts.push_back(p2);

  // A track close to the straight hop isn't a detour, so drive it direct; a
  // short-hop reversal is the same call, since no fillet can round a u-turn.
  double max_dev = 0.0;
  for (size_t i = 1; i + 1 < pts.size(); ++i) {
    max_dev = std::max(max_dev, pts[i].Distance(F2CLineString(p1, p2)));
  }
  bool direct = max_dev < robot.GetDirectHopMaxDev();
  if (!direct) {
    const double reversal = F2CPoint::GetAngleDiffAbs(ang1, ang2);
    const double hop = p1.Distance(p2);
    direct = reversal > robot.GetReversalSweep() && hop < robot.GetUturnMaxHop();
  }
  const auto checked = [&](const F2CPath& candidate) {
    // A valid connection must meet both swath headings as well as its geometry.
    // Otherwise a terminal straight can leave a square corner at the swath.
    const bool headings_match = candidate.size() > 0 &&
        F2CPoint::GetAngleDiffAbs(candidate[0].angle, ang1) < 1e-4 &&
        F2CPoint::GetAngleDiffAbs(candidate.back().angle, ang2) < 1e-4;

    if (field) {
      detail::FreeSpace free(*field);
      if (headings_match && detail::ValidPath(candidate, robot, &free)) return candidate;
      return detail::SearchConnection(robot, turn, p1, ang1, p2, ang2, free);
    }
    if (headings_match && detail::ValidPath(candidate, robot)) return candidate;
    auto corridor = F2CCells::buffer(F2CLineString(pts),
        robot.GetMaxCornerCut() + 2.0 * robot.GetMinTurningRadius());
    detail::FreeSpace free(corridor);
    return detail::SearchConnection(robot, turn, p1, ang1, p2, ang2, free);
  };
  if (direct) return checked(turn.CreateTurn(robot, p1, ang1, p2, ang2));

  // Otherwise follow the detour, rounding its corners, rather than cutting
  // across ground the route deliberately routed around.
  const std::vector<F2CPoint> poly =
      F2CLineString(pts).simplify(robot.GetTrackSimplifyTol()).ToVectorPoint();
  if (poly.size() < 2) {
    return {};
  }

  F2CPath path;
  AppendRoundedTrack(path, poly, robot, turn, robot.GetMaxCornerCut(),
      turn.HasContinuousCurvature(), ang1, ang2);
  return checked(path);
}

double PathPlanning::GetSmoothTurningRadius(const F2CRobot& robot) {
  return robot.GetSmoothTurningRadius();
}

}  // namespace autonomy::planning::planner::fields2cover::pp
