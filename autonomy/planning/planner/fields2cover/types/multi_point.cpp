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
 * @file multi_point.cpp
 * @brief Fields2Cover MultiPoint implementation.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */



#include "autonomy/planning/planner/fields2cover/types/multi_point.hpp"

namespace autonomy::planning::planner::fields2cover::types {

MultiPoint::MultiPoint() {
  data_ = std::shared_ptr<OGRMultiPoint>(
    static_cast<OGRMultiPoint*>(
      OGRGeometryFactory::createGeometry(wkbMultiPoint)),
    [](OGRMultiPoint* f) {OGRGeometryFactory::destroyGeometry(f);});
}

MultiPoint::MultiPoint(const std::vector<Point>& ps) {
  for (auto&& p : ps) {
    this->AddGeometry(p);
  }
}

MultiPoint::MultiPoint(const std::initializer_list<Point>& ps) {
  for (auto&& p : ps) {
    this->AddGeometry(p);
  }
}

size_t MultiPoint::size() const {
  return IsEmpty() ? 0 : this->data_->getNumGeometries();
}

std::vector<Point> MultiPoint::ToVectorPoint() const {
  std::vector<Point> points;
  points.reserve(this->size());
  for (size_t i = 0; i < this->size(); ++i) {
    points.emplace_back(this->GetGeometry(i));
  }
  return points;
}

void MultiPoint::GetGeometry(size_t i, Point& point) {
  if (i >= this->size()) {
    throw std::out_of_range(
        "MultiPoint does not contain point " + std::to_string(i));
  }
  point = Point(data_->getGeometryRef(i), EmptyDestructor());
}

void MultiPoint::GetGeometry(size_t i, Point& point) const {
  if (i >= this->size()) {
    throw std::out_of_range(
        "MultiPoint does not contain point " + std::to_string(i));
  }
  point = Point(data_->getGeometryRef(i), EmptyDestructor());
}

Point MultiPoint::GetGeometry(size_t i) {
  if (i >= this->size()) {
    throw std::out_of_range(
        "MultiPoint does not contain point " + std::to_string(i));
  }
  return Point(data_->getGeometryRef(i));
}

const Point MultiPoint::GetGeometry(size_t i) const {
  if (i >= this->size()) {
    throw std::out_of_range(
        "MultiPoint does not contain point " + std::to_string(i));
  }
  return Point(data_->getGeometryRef(i));
}

const Point MultiPoint::GetFirstPoint() const {
  return GetGeometry(0);
}

const Point MultiPoint::GetLastPoint() const {
  return GetGeometry(size() - 1);
}


void MultiPoint::SetGeometry(size_t i, const Point& p) {
    OGRPoint* point = DownCast<OGRPoint*>(data_->getGeometryRef(i));
    point->setX(p.GetX());
    point->setY(p.GetY());
    point->setZ(p.GetZ());
}


void MultiPoint::AddGeometry(const Point& p) {
  this->AddPoint(p);
}

void MultiPoint::AddPoint(const Point& p) {
  this->AddPoint(p.GetX(), p.GetY(), p.GetZ());
}

void MultiPoint::AddPoints(const MultiPoint& ps) {
  for (auto&& p : ps) {
    this->AddPoint(p);
  }
}

void MultiPoint::AddPoint(double x, double y, double z) {
  OGRPoint p(x, y, z);
  this->data_->addGeometry(&p);
}


void MultiPoint::operator*=(double b) {
  for (auto&& p : *this) {
    p *= b;
  }
}

double MultiPoint::GetInAngle(size_t i) const {
  // First point does not have In Angle
  if (0 == i) {
    throw std::invalid_argument(
        "MultiPoint::GetInAngle not defined for first point");
  }
  return (GetGeometry(i)-GetGeometry(i-1)).GetAngleFromPoint();
}

double MultiPoint::GetOutAngle(size_t i) const {
  // Last point does not have Out Angle
  if (i >= size() - 1) {
    throw std::invalid_argument(
        "MultiPoint::GetOutAngle not defined for last point");
  }
  return (GetGeometry(i+1)-GetGeometry(i)).GetAngleFromPoint();
}

double MultiPoint::GetPointAngle(size_t i) const {
  if (2 > size()) {
    throw std::invalid_argument(
        "MultiPoint::GetPointAngle not defined when size() < 2");
  }
  if (i == 0) {
    return GetOutAngle(i);
  } else if (i == size() - 1) {
    return GetInAngle(i);
  }
  return GetAngleAvg(GetInAngle(i), GetOutAngle(i));
}



}  // namespace autonomy::planning::planner::fields2cover::types

