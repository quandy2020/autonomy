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
 * @file route.cpp
 * @brief Fields2Cover Route implementation.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */



#include "autonomy/planning/planner/fields2cover/types/route.hpp"

namespace autonomy::planning::planner::fields2cover::types {

const std::vector<Swaths>& Route::GetVectorSwaths() const {
  return this->v_swaths_;
}

Swaths& Route::GetSwaths(size_t i) {
  return this->v_swaths_[i];
}

const Swaths& Route::GetSwaths(size_t i) const {
  return this->v_swaths_[i];
}

void Route::SetSwaths(size_t i, const Swaths& s) {
  this->v_swaths_[i] = s;
}

void Route::AddSwaths(const Swaths& s) {
  if (this->SizeConnections() == 0) {
    this->connections_.emplace_back();
  }
  if (this->SizeConnections() > this->SizeVectorSwaths()) {
    this->v_swaths_.emplace_back(s);
  } else {
    this->v_swaths_.back().Append(s);
  }
}

void Route::AddConnectedSwaths(const MultiPoint& mp, const Swaths& s) {
  this->AddConnection(mp);
  this->AddSwaths(s);
}

void Route::AddSwath(const Swath& s) {
  size_t nc = this->SizeConnections();
  const size_t ns = this->SizeVectorSwaths();
  if (nc == 0) {
    this->connections_.emplace_back();
    ++nc;
  }
  if (nc > ns || ns == 0) {
    this->v_swaths_.emplace_back();
  }
  this->v_swaths_.back().emplace_back(s);
}

void Route::AddSwath(const Swath& s, Graph2D& g) {
  if (!this->IsEmpty()) {
    this->AddConnection(g.ShortestPath(this->EndPoint(), s.StartPoint()));
  }
  this->AddSwath(s);
}

void Route::AddConnection() {
  if (this->SizeConnections() <= this->SizeVectorSwaths()) {
    this->connections_.emplace_back();
  }
}

void Route::AddConnection(const Point& p) {
  this->AddConnection();
  this->connections_.back().AddPoint(p);
}

void Route::AddConnection(const MultiPoint& mp) {
  if (mp.size() < 1) {
    this->AddConnection();
  }
  for (auto&& p : mp) {
    this->AddConnection(p);
  }
}
void Route::AddConnection(const std::vector<Point>& vp) {
  if (vp.size() < 1) {
    this->AddConnection();
  }
  for (auto&& p : vp) {
    this->AddConnection(p);
  }
}

MultiPoint& Route::GetLastConnection() {
  return this->connections_.back();
}

const MultiPoint& Route::GetLastConnection() const {
  return this->connections_.back();
}

Swaths& Route::GetLastSwaths() {
  return this->v_swaths_.back();
}

const Swaths& Route::GetLastSwaths() const {
  return this->v_swaths_.back();
}

const std::vector<MultiPoint>& Route::GetConnections() const {
  return this->connections_;
}

MultiPoint& Route::GetConnection(size_t i) {
  return this->connections_[i];
}

const MultiPoint& Route::GetConnection(size_t i) const {
  return this->connections_[i];
}

void Route::SetConnection(size_t i, const MultiPoint& mp) {
  this->connections_[i] = mp;
}

Point Route::StartPoint() const {
  if (SizeConnections() > 0 && connections_[0].size() > 0) {
    return connections_[0].GetFirstPoint();
  } else if (SizeVectorSwaths() > 0 && v_swaths_[0].size() > 0) {
    return v_swaths_[0][0].StartPoint();
  }
  return {};
}

Point Route::EndPoint() const {
  if (SizeConnections() > SizeVectorSwaths() &&
      connections_.back().size() > 0) {
    return connections_.back().GetLastPoint();
  } else if (SizeVectorSwaths() > 0 && v_swaths_.back().size() > 0) {
    return v_swaths_.back().back().EndPoint();
  }
  return {};
}

size_t Route::SizeVectorSwaths() const {
  return this->v_swaths_.size();
}

size_t Route::SizeConnections() const {
  return this->connections_.size();
}

double Route::Length() const {
  return AsLineString().Length();
}

LineString Route::AsLineString() const {
  if (IsEmpty()) {
    return LineString();
  }
  LineString line;
  if (connections_.size() > 0) {
    for (auto&& point : connections_[0]) {
      line.AddPoint(point);
    }
  }
  for (size_t i = 0; i < v_swaths_.size(); ++i) {
    for (auto&& swath : v_swaths_[i]) {
      for (const auto& point : swath.GetPath()) {
        line.AddPoint(point);
      }
    }
    if ((connections_.size() > i + 1)) {
      for (auto&& point : connections_[i+1]) {
        line.AddPoint(point);
      }
    }
  }
  return line;
}

bool Route::IsEmpty() const {
  return v_swaths_.empty() && connections_.empty();
}

void Route::Reverse() {
  // Connections are stored before the swaths they lead into, and the last one
  // is optional. Adding it keeps the connections paired with the right swaths
  // once the order is flipped.
  if (this->SizeConnections() == this->SizeVectorSwaths()) {
    this->connections_.emplace_back();
  }
  std::reverse(this->connections_.begin(), this->connections_.end());
  for (auto&& c : this->connections_) {
    MultiPoint reversed;
    for (size_t i = c.size(); i > 0; --i) {
      reversed.AddPoint(c.GetGeometry(i - 1));
    }
    c = reversed;
  }

  if (this->SizeConnections() > this->SizeVectorSwaths() &&
      this->connections_.back().IsEmpty()) {
    this->connections_.pop_back();
  }

  std::reverse(this->v_swaths_.begin(), this->v_swaths_.end());
  for (auto&& s : this->v_swaths_) {
    s.Reverse();
    for (auto&& swath : s) {
      swath.Reverse();
    }
  }
}

Route Route::Clone() const {
  Route new_r;
  for (auto&& s : this->v_swaths_) {
    new_r.v_swaths_.emplace_back(s.Clone());
  }
  for (auto&& c : this->connections_) {
    new_r.connections_.emplace_back(c.Clone());
  }
  return new_r;
}


}  // namespace autonomy::planning::planner::fields2cover::types

