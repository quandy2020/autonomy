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
 * @file turning_base.cpp
 * @brief Fields2Cover turning base implementation.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */



#include "autonomy/planning/planner/fields2cover/path_planning/turning_base.hpp"

namespace autonomy::planning::planner::fields2cover::pp {

std::vector<double> TurningBase::TransformToNormalTurn(
    const F2CPoint& start_pos, double start_angle,
    const F2CPoint& end_pos, double end_angle) {
  double dist_start_end = start_pos.Distance(end_pos);
  auto dir = end_pos - start_pos;
  auto angle = F2CPoint::Mod2pi(dir.GetAngleFromPoint());
  bool inverted {false};
  start_angle = F2CPoint::Mod2pi(start_angle - angle);
  end_angle = F2CPoint::Mod2pi(end_angle - angle);
  if (start_angle > boost::math::constants::pi<double>()) {
    start_angle = F2CPoint::Mod2pi(-start_angle);
    end_angle = F2CPoint::Mod2pi(-end_angle);
    inverted = true;
  }
  return {dist_start_end, angle, start_angle, end_angle,
    static_cast<double>(inverted)};
}


F2CPath TurningBase::CreateTurn(const F2CRobot& robot,
    const F2CPoint& start_pos, double start_angle,
    const F2CPoint& end_pos, double end_angle) {
  auto turn_values =
    TransformToNormalTurn(start_pos, start_angle, end_pos, end_angle);
  double dist_start_end = turn_values[0];
  double rot_angle = turn_values[1];
  double start_angle_t = turn_values[2];
  double end_angle_t = turn_values[3];
  double inverted = turn_values[4];

  F2CPath path;
  if (using_cache) {
    path = CreateTurnIfNotCached(robot,
        dist_start_end, start_angle_t, end_angle_t);
  } else {
    path = CreateSimpleTurn(robot,
        dist_start_end, start_angle_t, end_angle_t);
  }
  if (path.size() <= 1) {return F2CPath();}

  if (inverted) {
    std::for_each(path.begin(), path.end(), [] (auto& s) {
        s.point.SetY(-s.point.GetY());
        s.angle = F2CPoint::Mod2pi(-s.angle);});
  }
  for (auto&& s : path) {
    s.point = F2CPoint(.0, .0).RotateFromPoint(rot_angle, s.point) + start_pos;
    s.angle = F2CPoint::Mod2pi(s.angle + rot_angle);
  }

  CorrectPath(path, start_pos, end_pos);
  return path;
}

void TurningBase::CorrectPath(F2CPath& path, const F2CPoint& start_pos,
    const F2CPoint& end_pos, float max_error_dist) {
  if (path.size() < 2) {return;}

  auto IsNear = [max_error_dist](const F2CPoint& a, const F2CPoint& b) {
      return (a.Distance(b) < max_error_dist);
  };
  if (IsNear(path[0].point, start_pos)) {
    path[0].point = start_pos;
  }
  if (IsNear(path.back().point, end_pos)) {
    path.back().point = end_pos;
  }
}


F2CPath TurningBase::CreateTurnIfNotCached(const F2CRobot& robot,
    double dist_start_end, double start_angle, double end_angle) {
  std::vector<int> v_turn {
        static_cast<int>(1e3 * robot.GetMaxCurv()),
        static_cast<int>(1e3 * robot.GetMaxDiffCurv()),
        static_cast<int>(1e3 * dist_start_end),
        static_cast<int>(1e3 * start_angle),
        static_cast<int>(1e3 * end_angle)
  };
  auto it = path_cache_.find(v_turn);
  if (it != path_cache_.end()) {
    return it->second;
  }
  auto path = CreateSimpleTurn(robot, dist_start_end, start_angle, end_angle);
  path_cache_.insert({v_turn, path});
  return path;
}


bool TurningBase::IsTurnValid(const types::Path& path,
    double dist_start_end, double end_angle,
    double max_dist, double max_rot_error) {
  for (auto&& s : path) {
    if (s.point.GetY() < -max_dist) {
      return false;
    }
  }
  F2CPoint p_end = path.AtEnd();
  return (cos(path.back().angle - end_angle) >= 1 - max_rot_error) &&
    (fabs(p_end.GetX() - dist_start_end) < max_dist) &&
    (fabs(p_end.GetY()) < max_dist);
}

bool TurningBase::HasContinuousCurvature() const {
  return false;
}

double TurningBase::GetDiscretization() const {
  return this->discretization;
}

void TurningBase::SetDiscretization(double d) {
  this->discretization = d;
}

bool TurningBase::GetUsingCache() const {
  return this->using_cache;
}

void TurningBase::SetUsingCache(bool c) {
  this->using_cache = c;
}


}  // namespace autonomy::planning::planner::fields2cover::pp

