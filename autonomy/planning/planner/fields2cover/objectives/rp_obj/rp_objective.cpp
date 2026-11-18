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
 * @file rp_objective.cpp
 * @brief Fields2Cover rp objective implementation.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */



#include "autonomy/planning/planner/fields2cover/objectives/rp_obj/rp_objective.hpp"

namespace autonomy::planning::planner::fields2cover::obj {

double RPObjective::ComputeCost(
    const F2CPoint& p1, const F2CPoint& p2) {
  return p1.Distance(p2);
}

double RPObjective::ComputeCost(
    const F2CPoint& p1, double ang1, const F2CPoint& p2) {
  return ComputeCost(p1, p2);
}


double RPObjective::ComputeCost(
    const F2CPoint& p1, const F2CPoint& p2, double ang2) {
  return ComputeCost(p1, p2);
}

double RPObjective::ComputeCost(
    const F2CPoint& p1, double ang1, const F2CPoint& p2, double ang2) {
  return ComputeCost(p1, p2);
}

double RPObjective::ComputeCost(const F2CSwath& s, const F2CPoint& p) {
  return ComputeCost(s.EndPoint(), s.GetOutAngle(), p);
}

double RPObjective::ComputeCost(const F2CSwath& s1, const F2CSwath& s2) {
  return this->ComputeCost(
      s1.EndPoint(), s1.GetOutAngle(), s2.StartPoint(), s2.GetInAngle());
}

double RPObjective::ComputeCost(
    const F2CSwath& s, const F2CPoint& p, double ang) {
  return ComputeCost(s.EndPoint(), s.GetOutAngle(), p, ang);
}

double RPObjective::ComputeCost(const F2CPoint& p, const F2CSwath& s) {
  return ComputeCost(p, s.StartPoint(), s.GetInAngle());
}

double RPObjective::ComputeCost(
    const F2CPoint& p, double ang, const F2CSwath& s) {
  return ComputeCost(p, ang, s.StartPoint(), s.GetInAngle());
}

double RPObjective::ComputeCost(const std::vector<F2CPoint>& ps) {
  auto dist {0.0};
  for (size_t i = 1; i < ps.size(); ++i) {
    dist += ComputeCost(ps[i-1], ps[i]);
  }
  return dist;
}

double RPObjective::ComputeCost(const F2CMultiPoint& ps) {
  if (ps.IsEmpty()) {
    return 0.0;
  }
  auto dist {0.0};
  for (size_t i = 1; i < ps.size(); ++i) {
    dist += ComputeCost(ps.GetGeometry(i-1), ps.GetGeometry(i));
  }
  return dist;
}

double RPObjective::ComputeCost(const F2CSwath& s, const F2CMultiPoint& ps) {
  if (ps.IsEmpty()) {
    return 0.0;
  }
  return ComputeCost(s.EndPoint(), s.GetOutAngle(), ps.GetGeometry(0));
}

double RPObjective::ComputeCost(const F2CSwaths& s, const F2CMultiPoint& ps) {
  if (ps.IsEmpty()) {
    return 0.0;
  }
  return ComputeCost(s.back(), ps);
}

double RPObjective::ComputeCost(const F2CMultiPoint& ps, const F2CSwath& s) {
  if (ps->IsEmpty()) {
    return 0.0;
  }
  return ComputeCost(ps.GetGeometry(ps.size()-1),
      s.StartPoint(), s.GetInAngle());
}

double RPObjective::ComputeCost(const F2CMultiPoint& ps, const F2CSwaths& s) {
  return ComputeCost(ps, s[0]);
}

double RPObjective::ComputeCost(const F2CSwath& s) {
  double dist {0.0};
  for (size_t i = 1; i < s.NumPoints(); ++i) {
    dist += ComputeCost(s.GetPoint(i-1), s.GetPoint(i));
  }
  return dist;
}

double RPObjective::ComputeCost(const F2CSwaths& swaths) {
  double dist {0.0};
  for (auto&& s : swaths) {
    dist += ComputeCost(s);
  }
  for (size_t i = 1; i < swaths.size(); ++i) {
    dist += ComputeCost(swaths[i-1].EndPoint(), swaths[i-1].GetOutAngle(),
        swaths[i].StartPoint(), swaths[i].GetInAngle());
  }
  return dist;
}

double RPObjective::ComputeCost(const F2CRoute& r) {
  double dist {0.0};
  for (auto&& s : r.GetVectorSwaths()) {
    dist += ComputeCost(s);
  }
  for (auto&& s : r.GetConnections()) {
    dist += ComputeCost(s);
  }
  for (size_t i = 0; i < r.SizeVectorSwaths(); ++i) {
    dist += ComputeCost(r.GetConnection(i), r.GetSwaths(i));
    if (i + 1 < r.SizeConnections()) {
      dist += ComputeCost(r.GetSwaths(i), r.GetConnection(i + 1));
    }
  }
  return dist;
}

}  // namespace autonomy::planning::planner::fields2cover::obj

