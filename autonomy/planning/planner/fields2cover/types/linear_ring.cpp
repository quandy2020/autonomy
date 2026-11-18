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
 * @file linear_ring.cpp
 * @brief Fields2Cover LinearRing implementation.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */



#include <vector>
#include "autonomy/planning/planner/fields2cover/types/linear_ring.hpp"
#include "autonomy/planning/planner/fields2cover/types/line_string.hpp"

namespace autonomy::planning::planner::fields2cover::types {

LinearRing::LinearRing() {
  this->data_ = std::shared_ptr<OGRLinearRing>(
    static_cast<OGRLinearRing*>(
      OGRGeometryFactory::createGeometry(wkbLinearRing)),
    [](OGRLinearRing* f) {OGRGeometryFactory::destroyGeometry(f);});
}

LinearRing::LinearRing(const std::vector<Point>& ps) {
  for (auto&& p : ps) {
    this->AddPoint(p);
  }
}

LinearRing::LinearRing(const std::initializer_list<Point>& ps) {
  for (auto&& p : ps) {
    this->AddPoint(p);
  }
}

void LinearRing::operator*=(double b) {
  for (auto&& p : *this) {
    p *= b;
  }
}

double LinearRing::GetX(size_t i) const {
  return this->data_->getX(i);
}

double LinearRing::GetY(size_t i) const {
  return this->data_->getY(i);
}
double LinearRing::GetZ(size_t i) const {
  return this->data_->getZ(i);
}
double LinearRing::Length() const {
  return this->data_->get_Length();
}

void LinearRing::ReversePoints() {
  this->data_->reversePoints();
}

size_t LinearRing::size() const {
  return IsEmpty() ? 0 : this->data_->getNumPoints();
}

void LinearRing::GetGeometry(size_t i, Point& point) {
  if (i >= this->size()) {
    throw std::out_of_range(
        "Error GetGeometry: LinearRing does not contain point " +
        std::to_string(i));
  }
  data_->getPoint(i, point.get());
}

void LinearRing::GetGeometry(size_t i, Point& point) const {
  if (i >= this->size()) {
    throw std::out_of_range(
        "Error GetGeometry: LinearRing does not contain point " +
        std::to_string(i));
  }
  data_->getPoint(i, point.get());
}

Point LinearRing::GetGeometry(size_t i) {
  if (i >= this->size()) {
    throw std::out_of_range(
        "Error GetGeometry: LinearRing does not contain point " +
        std::to_string(i));
  }
  OGRPoint point;
  data_->getPoint(i, &point);
  return Point(point);
}

const Point LinearRing::GetGeometry(size_t i) const {
  if (i >= this->size()) {
    throw std::out_of_range(
        "Error GetGeometry: LinearRing does not contain point " +
        std::to_string(i));
  }
  return Point(data_->getX(i), data_->getY(i), data_->getZ(i));
}

void LinearRing::SetGeometry(size_t i, const Point& p) {
  data_->setPoint(i, p.GetX(), p.GetY(), p.GetZ());
}

void LinearRing::AddGeometry(const Point& p) {
  this->AddPoint(p);
}

void LinearRing::AddPoint(double x, double y, double z) {
  data_->addPoint(x, y, z);
}

void LinearRing::AddPoint(const Point& p) {
  data_->addPoint(p.GetX(), p.GetY(), p.GetZ());
}

const Point LinearRing::StartPoint() const {
  return GetGeometry(0);
}

const Point LinearRing::EndPoint() const {
  return GetGeometry(size()-1);
}

bool LinearRing::IsClockwise() const {
  return data_->isClockwise();
}

bool LinearRing::IsClosed() const {
  return data_->IsEmpty() || data_->get_IsClosed();
}

LinearRing& LinearRing::CloseRing() {
  if (!this->IsClosed()) {
    this->AddPoint(this->StartPoint());
  }
  return *this;
}

Point LinearRing::ClosestPointTo(const Point& p) const {
  std::vector<double> dist;
  std::vector<Point> ps;
  for (size_t i = 0; i < this->size() - 1; ++i) {
    ps.emplace_back(p.ClosestPointInSegment(this->at(i), this->at(i+1)));
    dist.emplace_back(ps.back().Distance(p));
  }
  return ps[std::min_element(dist.begin(), dist.end()) - dist.begin()];
}

LinearRing& LinearRing::RemovePoint(size_t i) {
  this->data_->removePoint(i);
  return *this;
}

LineString LinearRing::GetSegment(size_t i) const {
  return LineString(this->at(i), this->at(i+1));
}

LineString LinearRing::GetLastSegment() const {
  return this->GetSegment(this->size()-2);
}

double LinearRing::SegmentLength(size_t i) const {
  return this->at(i).Distance(this->at(i+1));
}

double LinearRing::SegmentAng(size_t i) const {
  return (this->at(i + 1) - this->at(i)).GetAngleFromPoint();
}

LinearRing& LinearRing::FilterSelfIntersections() {
  if (this->size() <= 3) {
    return *this;
  }
  for (size_t i = 0; i + 2 < size(); ++i) {
    auto s_i = this->GetSegment(i);
    auto s_j = this->GetSegment(i+1);
    if (s_i.within(s_j) || s_i.contains(s_j)) {
      this->RemovePoint(i+1);
    }
  }
  for (size_t i = 0; i + 2 < size(); ++i) {
    for (size_t j = this->size() - 2; j > i + 1; --j) {
      if (i == 0 && j + 2 == size()) {continue;}
      if (this->GetSegment(i).intersects(this->GetSegment(j))) {
        auto p_i = Point::IntersectionOfLines(at(i), at(i+1), at(j), at(j+1));
        for (size_t k = j; k > i + 1; --k) {
          this->RemovePoint(k);
        }
        this->SetGeometry(i+1, p_i);
        break;
      }
    }
  }
  return *this;
}

LinearRing LinearRing::GetParallelLine(const std::vector<double>& d) const {
  if (this->size() != d.size() + 1) {
    throw std::invalid_argument(
        "Error LinearRing::GetParallelLine. "  \
        "LinearRing::size() and d.size() + 1 are not equal");
  }

  std::vector<Point> ps0, ps1;
  for (size_t i = 0; i < d.size(); ++i) {
    double ang = this->SegmentAng(i);
    ps0.emplace_back(at(i).GetPointFromAngle(ang + 0.5 * M_PI, d[i]));
    ps1.emplace_back(at(i+1).GetPointFromAngle(ang + 0.5 * M_PI, d[i]));
  }

  LinearRing ring;
  ring.AddPoint(Point::IntersectionOfLines(
        ps0.back(), ps1.back(), ps0[0], ps1[0]));
  for (size_t i = 1; i < ps0.size(); ++i) {
    ring.AddPoint(Point::IntersectionOfLines(
          ps0[i-1], ps1[i-1], ps0[i], ps1[i]));
  }
  ring.AddPoint(ring[0]);
  ring.FilterSelfIntersections();
  return ring;
}

LinearRing LinearRing::GetParallelLine(double d) const {
  return GetParallelLine(std::vector<double>(this->size()-1, d));
}

LinearRing& LinearRing::BufferOutwards(const std::vector<double>& d) {
  auto ds = d;
  if (!this->IsClockwise()) {
    for (auto&& d_val : ds) {
      d_val *= -1.0;
    }
  }
  *this = this->GetParallelLine(ds);
  return *this;
}

LinearRing& LinearRing::BufferInwards(const std::vector<double>& d) {
  auto ds = d;
  if (this->IsClockwise()) {
    for (auto&& d_val : ds) {
      d_val *= -1.0;
    }
  }
  *this = this->GetParallelLine(ds);
  return *this;
}


}  // namespace autonomy::planning::planner::fields2cover::types

