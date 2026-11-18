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
 * @file path.cpp
 * @brief Fields2Cover Path implementation.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */



#include <algorithm>
#include <numeric>
#include "autonomy/planning/planner/fields2cover/utils/spline.hpp"
#include "autonomy/planning/planner/fields2cover/types/path.hpp"

namespace autonomy::planning::planner::fields2cover::types {

PathState& Path::GetState(size_t i) {
  return this->states_[i];
}

const PathState& Path::GetState(size_t i) const {
  return this->states_[i];
}

void Path::SetState(size_t i, const PathState& ps) {
  this->states_[i] = ps;
}

void Path::AddState(const PathState& ps) {
  this->states_.emplace_back(ps);
}

void Path::AddState(const Point& p, double ang, double len,
    PathDirection dir, PathSectionType type, double vel) {
  PathState ps;
  ps.point = p;
  ps.angle = ang;
  ps.len = len;
  ps.dir = dir;
  ps.type = type;
  ps.velocity = vel;
  this->states_.emplace_back(ps);
}

std::vector<PathState>& Path::GetStates() {
  return this->states_;
}

const std::vector<PathState>& Path::GetStates() const {
  return this->states_;
}

void Path::SetStates(const std::vector<PathState>& v_ps) {
  this->states_ = v_ps;
}

double Path::GetTaskTime() const {
  return std::accumulate(this->begin(), this->end(), 0.0,
      [] (double d, const PathState& s) {return s.len / s.velocity + d;});
}

std::vector<PathState>::const_iterator Path::cbegin() const {
  return this->states_.begin();
}

std::vector<PathState>::const_iterator Path::cend() const {
  return this->states_.end();
}

std::vector<PathState>::const_iterator Path::begin() const {
  return this->cbegin();
}

std::vector<PathState>::const_iterator Path::end() const {
  return this->cend();
}

std::vector<PathState>::iterator Path::begin() {
  return this->states_.begin();
}

std::vector<PathState>::iterator Path::end() {
  return this->states_.end();
}

const PathState& Path::operator[](size_t idx) const {
  return this->states_[idx];
}

PathState& Path::operator[](size_t idx) {
  return this->states_[idx];
}

const PathState& Path::back() const {
  return this->states_.back();
}

PathState& Path::back() {
  return this->states_.back();
}

Path& Path::operator+=(const Path& path) {
  for (auto&& new_state : path) {
    this->states_.emplace_back(new_state);
  }
  return *this;
}

size_t Path::size() const {
  return states_.size();
}

double Path::GetDimMinX() const {
  return std::min_element(this->begin(), this->end(),
      [](const PathState& s1, const PathState& s2) {
        return s1.point.GetX() < s2.point.GetX();
      })->point.GetX();
}

double Path::GetDimMinY() const {
  return std::min_element(this->begin(), this->end(),
      [](const PathState& s1, const PathState& s2) {
        return s1.point.GetY() < s2.point.GetY();
      })->point.GetY();
}

double Path::GetDimMaxX() const {
  return std::max_element(this->begin(), this->end(),
      [](const PathState& s1, const PathState& s2) {
        return s1.point.GetX() < s2.point.GetX();
      })->point.GetX();
}

double Path::GetDimMaxY() const {
  return std::max_element(this->begin(), this->end(),
      [](const PathState& s1, const PathState& s2) {
        return s1.point.GetY() < s2.point.GetY();
      })->point.GetY();
}

void Path::MoveTo(const Point& p) {
  for (auto&& s : this->states_) {
    s.point = s.point + p;
  }
}

void Path::RotateFromPoint(const Point& p, double ang) {
  for (auto&& s : this->states_) {
    s.point = p.RotateFromPoint(ang, s.point);
    s.angle = s.point.Mod2pi(s.angle + ang);
  }
}

void Path::MirrorX() {
  for (auto&& s : this->states_) {
    s.point.SetX(-s.point.GetX());
    s.angle = s.point.Mod2pi(M_PI - s.angle);
  }
}
void Path::MirrorY() {
  for (auto&& s : this->states_) {
    s.point.SetY(-s.point.GetY());
    s.angle = s.point.Mod2pi(-s.angle);
  }
}

void Path::SetBackwardDir() {
  for (size_t i = 0; i < this->size(); ++i) {
    this->SetBackwardDir(i);
  }
}

void Path::SetBackwardDir(int i) {
  this->states_[i].dir = PathDirection::BACKWARD;
}

void Path::SetForwardDir() {
  for (size_t i = 0; i < this->size(); ++i) {
    this->SetForwardDir(i);
  }
}

void Path::SetForwardDir(int i) {
  this->states_[i].dir = PathDirection::FORWARD;
}

void Path::SetSwathType() {
  for (size_t i = 0; i < this->size(); ++i) {
    this->SetSwathType(i);
  }
}

void Path::SetSwathType(int i) {
  this->states_[i].type = PathSectionType::SWATH;
}

void Path::SetTurnType() {
  for (size_t i = 0; i < this->size(); ++i) {
    this->SetTurnType(i);
  }
}

void Path::SetTurnType(int i) {
  this->states_[i].type = PathSectionType::TURN;
}


double Path::Length() const {
  return std::accumulate(states_.begin(), states_.end(), 0.0,
      [](double d, const PathState& s) {return d + fabs(s.len);});
}

void Path::AppendSwath(const Swath& swath, double cruise_speed) {
  this->AppendSwath(swath, cruise_speed, PathSectionType::SWATH);
}

void Path::AppendSwath(
    const Swath& swath, double cruise_speed, PathSectionType type) {
  for (size_t i = 0; i < swath.NumPoints() - 1; ++i) {
    PathState s;
    s.point = swath.GetPoint(i);
    auto p_ang = Point(swath.GetPoint(i + 1).GetX() - swath.GetPoint(i).GetX(),
                       swath.GetPoint(i + 1).GetY() - swath.GetPoint(i).GetY());
    s.angle = p_ang.GetAngleFromPoint();
    s.len = swath.GetPoint(i + 1).Distance(swath.GetPoint(i));
    s.dir = PathDirection::FORWARD;
    s.type = type;
    s.velocity = cruise_speed;
    this->AddState(s);
  }
}

void Path::AppendStraight(const Point& start, const Point& end,
    double cruise_speed, PathSectionType type) {
  const double len = start.Distance(end);
  if (len < 1e-6) {
    return;
  }
  PathState s;
  s.point = start;
  s.angle = (end - start).GetAngleFromPoint();
  s.len = len;
  s.dir = PathDirection::FORWARD;
  s.type = type;
  s.velocity = cruise_speed;
  this->AddState(s);
}

double Path::MaxDistanceTo(const LineString& line) const {
  double worst = 0.0;
  for (auto&& s : this->states_) {
    worst = std::max(worst, s.point.Distance(line));
  }
  return worst;
}

PathState Path::at(double len) const {
  double total_length = this->Length();
  if (len > total_length) {
    return this->at(total_length);
  } else if (len < 0.0) {
    return this->at(0.0);
  }

  PathState state_at;
  for (size_t i = 0; i < this->size(); ++i) {
    double len_state = fabs(states_[i].len);
    if (len_state < len) {
      len -= len_state;
    } else {
      state_at = states_[i];
      Point p = states_[i].point;
      double ang = states_[i].angle;

      len *= static_cast<double>(states_[i].dir);
      state_at.point = Point(p.GetX() + len * cos(ang),
                             p.GetY() + len * sin(ang));
      break;
    }
  }
  return state_at;
}

Point Path::AtStart() const {
  return states_[0].point;
}

Point Path::AtEnd() const {
  return states_.back().AtEnd();
}

template<typename CharT>
class DecimalSeparator : public std::numpunct<CharT>{
 public:
  explicit DecimalSeparator(CharT Separator) : m_Separator(Separator) {}

 protected:
  CharT do_decimal_point() const {
    return m_Separator;
  }

 private:
  CharT m_Separator;
};

std::string to_string(double d, const int precision = 6) {
  std::stringstream ss;
  ss.precision(precision);
  ss.imbue(std::locale(std::locale(), new DecimalSeparator<char>('.')));
  ss << d;
  return ss.str();
}


std::string Path::SerializePath(size_t digit_precision) const {
  std::locale::global(std::locale::classic());
  std::string res = "";

  for (size_t i = 0; i < size(); ++i) {
    res += to_string(states_[i].point.GetX(), digit_precision) + " ";
    res += to_string(states_[i].point.GetY(), digit_precision) + " ";
    res += to_string(states_[i].point.GetZ(), digit_precision) + " ";
    res += to_string(states_[i].angle, digit_precision) + " ";
    res += to_string(states_[i].velocity, digit_precision) + " ";
    res += to_string(states_[i].len, digit_precision) + " ";
    res += to_string(static_cast<int>(states_[i].dir)) + " ";
    res += to_string(static_cast<int>(states_[i].type)) + "\n";
  }
  return res;
}

/**
 * @brief Discretize the swath and headland-swath sections of the path and
 *        return a new path
 * @param[in] step_size Discretization step in [m]
 * @return New path with swath and headland-swath sections now discretized
*/
Path Path::DiscretizeSwath(double step_size) const {
  Path new_path;
  for (auto&& s : this->states_) {
    if (s.type == PathSectionType::SWATH ||
        s.type == PathSectionType::HL_SWATH) {
      double n_steps = std::max(1.0, std::round(fabs(s.len / step_size)));
      Point start2end = s.AtEnd() - s.point;
      for (double j = 0.0; j < n_steps; j += 1.0) {
        PathState state = s;
        state.point = s.point + start2end * (j / n_steps);
        state.len /= n_steps;
        new_path.AddState(state);
      }
    } else {
      new_path.AddState(s);
    }
  }
  return new_path;
}

void Path::SaveToFile(const std::string& file, size_t precision) const {
  std::ofstream out(file);
  out << SerializePath(precision);
  out.close();
}

void Path::LoadFile(const std::string& file) {
  std::ifstream in(file);
  double x, y, z, ang, vel, len, dir, d_type;
  while (in >> x >> y >> z >> ang >> vel >> len >> dir >> d_type) {
    PathState s;
    s.point = Point(x, y, z);
    s.angle = ang;
    s.velocity = vel;
    s.len = len;
    s.dir = static_cast<autonomy::planning::planner::fields2cover::types::PathDirection>(dir);
    s.type = static_cast<autonomy::planning::planner::fields2cover::types::PathSectionType>(d_type);
    this->states_.emplace_back(s);
  }
  in.close();
}

Path& Path::populate(int number_points) {
  if (this->size() < 3) { return *this;}

  std::vector<double> x, y, ang_prov, old_vel, len;
  std::vector<PathDirection> old_dir;
  std::vector<PathSectionType> old_type;
  for (int i = 0; i < this->size(); ++i) {
    if (this->states_[i].len != 0.0) {
      x.emplace_back(this->states_[i].point.GetX());
      y.emplace_back(this->states_[i].point.GetY());
      ang_prov.emplace_back(this->states_[i].angle);
      len.emplace_back(this->states_[i].len);
      old_vel.emplace_back(this->states_[i].velocity);
      old_dir.emplace_back(this->states_[i].dir);
      old_type.emplace_back(this->states_[i].type);
    }
  }
  x.emplace_back(this->states_.back().AtEnd().GetX());
  y.emplace_back(this->states_.back().AtEnd().GetY());
  ang_prov.emplace_back(this->states_.back().angle);
  len.emplace_back(0.0);
  old_vel.emplace_back(this->states_.back().velocity);
  old_dir.emplace_back(this->states_.back().dir);
  old_type.emplace_back(this->states_.back().type);

  auto ang = autonomy::planning::planner::fields2cover::types::Point::GetAngContinuity(ang_prov);
  std::vector<double> t(len.size());
  std::partial_sum(len.begin(), len.end() - 1, t.begin() + 1);

  CubicSpline s_x(t, x);
  CubicSpline s_y(t, y);
  CubicSpline s_ang(t, ang);

  this->states_.clear();

  double step = t.back() / static_cast<double>(number_points);
  PathState state;
  for (size_t i = 0; i < number_points; ++i) {
    double d = i * step;
    state.point = Point(s_x(d), s_y(d));
    state.angle = Point::Mod2pi(s_ang(d));
    state.len = step;
    int it = std::lower_bound(t.begin(), t.end(), d + 1e-10) - t.begin() - 1;
    state.velocity = old_vel[it];
    state.dir = old_dir[it];
    state.type = old_type[it];
    this->AddState(state);
  }
  return *this;
}

Path& Path::reduce(double min_dist_equal) {
  size_t N = this->size();
  Path new_path;
  for (size_t i = 0; i < N;) {
    double new_len {this->states_[i].len};
    int j = i + 1;
    while (j < N &&
        this->states_[i].type == this->states_[j].type &&
        this->states_[i].dir == this->states_[j].dir &&
        this->states_[i].point.Distance(
          this->states_[j].point) < min_dist_equal) {
      new_len += this->states_[j].len;
      ++j;
    }
    PathState state;
    state.point = this->states_[i].point;
    state.angle = this->states_[i].angle;
    state.velocity = this->states_[i].velocity;
    state.len = new_len;
    state.dir = this->states_[i].dir;
    state.type = this->states_[i].type;
    new_path.states_.emplace_back(state);
    i = j;
  }
  *this = new_path;
  return *this;
}

LineString Path::ToLineString() const {
  LineString line;
  for (auto&& s : this->states_) {
    line.AddPoint(s.point);
  }
  line.AddPoint(this->AtEnd());
  return line;
}

Path& Path::discretize(double step_size) {
  if (this->size() > 1) {
    int n = static_cast<int>((10.0 / step_size) * this->Length());
    this->populate(n);
    this->reduce(step_size);
  }
  return *this;
}

}  // namespace autonomy::planning::planner::fields2cover::types

