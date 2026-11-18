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
 * @file line_string.cpp
 * @brief Fields2Cover LineString implementation.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */



#include "autonomy/planning/planner/fields2cover/types/line_string.hpp"

namespace autonomy::planning::planner::fields2cover::types {

LineString::LineString() {
  data_ = std::shared_ptr<OGRLineString>(
    static_cast<OGRLineString*>(
      OGRGeometryFactory::createGeometry(wkbLineString)),
    [](OGRLineString* f) {OGRGeometryFactory::destroyGeometry(f);});
}

LineString::LineString(const LinearRing& ring) {
  for (auto&& p : ring) {
    this->AddPoint(p);
  }
}

LineString::LineString(const std::vector<Point>& ps) {
  for (auto&& p : ps) {
    this->AddPoint(p);
  }
}

LineString::LineString(const Point& p1, const Point& p2) {
  this->AddPoint(p1);
  this->AddPoint(p2);
}

LineString::LineString(const std::initializer_list<Point>& ps) {
  for (auto&& p : ps) {
    this->AddPoint(p);
  }
}

double LineString::X(size_t i) const {return data_->getX(i);}
double LineString::Y(size_t i) const {return data_->getY(i);}
double LineString::Z(size_t i) const {return data_->getZ(i);}
double LineString::GetX(size_t i) const {return data_->getX(i);}
double LineString::GetY(size_t i) const {return data_->getY(i);}
double LineString::GetZ(size_t i) const {return data_->getZ(i);}
double LineString::Length() const {return this->data_->get_Length();}
void LineString::ReversePoints() { this->data_->reversePoints();}
size_t LineString::size() const {
  return IsEmpty() ? 0 : this->data_->getNumPoints();
}

std::vector<Point> LineString::ToVectorPoint() const {
  std::vector<Point> points;
  points.reserve(this->size());
  for (size_t i = 0; i < this->size(); ++i) {
    points.emplace_back(this->GetGeometry(i));
  }
  return points;
}


void LineString::GetGeometry(size_t i, Point& point) {
  if (i >= this->size()) {
    throw std::out_of_range(
        "Error GetGeometry: LinearString does not contain point " +
        std::to_string(i));
  }
  data_->getPoint(i, point.get());
}

void LineString::GetGeometry(size_t i, Point& point) const {
  if (i >= this->size()) {
    throw std::out_of_range(
        "Error GetGeometry: LinearString does not contain point " +
        std::to_string(i));
  }
  data_->getPoint(i, point.get());
}

Point LineString::GetGeometry(size_t i) {
  if (i >= this->size()) {
    throw std::out_of_range(
        "Error GetGeometry: LinearString does not contain point " +
        std::to_string(i));
  }
  OGRPoint point;
  data_->getPoint(i, &point);
  return Point(point);
}

const Point LineString::GetGeometry(size_t i) const {
  if (i >= this->size()) {
    throw std::out_of_range(
        "Error GetGeometry: LinearString does not contain point " +
        std::to_string(i));
  }
  return Point(data_->getX(i), data_->getY(i), data_->getZ(i));
}

void LineString::SetGeometry(size_t i, const Point& p) {
  data_->setPoint(i, p.GetX(), p.GetY(), p.GetZ());
}

void LineString::AddGeometry(const Point& p) {
  this->AddPoint(p);
}


void LineString::operator*=(double b) {
  for (auto&& p : *this) {
    p *= b;
  }
}

void LineString::AddPoint(double x, double y, double z) {
  data_->addPoint(x, y, z);
}
void LineString::AddPoint(const Point& p) {
  data_->addPoint(p.GetX(), p.GetY(), p.GetZ());
}

const Point LineString::StartPoint() const {
  return GetGeometry(0);
}

const Point LineString::EndPoint() const {
  return GetGeometry(size()-1);
}

double LineString::StartAngle() const {
  return (GetGeometry(1) - StartPoint()).GetAngleFromPoint();
}

double LineString::EndAngle() const {
  return (EndPoint() - GetGeometry(size() - 2)).GetAngleFromPoint();
}

Point LineString::ClosestPointTo(const Point& p) const {
  std::vector<double> dist;
  std::vector<Point> ps;
  for (size_t i = 0; i < this->size() - 1; ++i) {
    ps.emplace_back(p.ClosestPointInSegment(this->at(i), this->at(i+1)));
    dist.emplace_back(ps.back().Distance(p));
  }
  return ps[std::min_element(dist.begin(), dist.end()) - dist.begin()];
}


}  // namespace autonomy::planning::planner::fields2cover::types

