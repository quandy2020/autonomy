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
 * @file cell.cpp
 * @brief Fields2Cover Cell implementation.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */



#include "autonomy/planning/planner/fields2cover/types/cell.hpp"
#include "autonomy/planning/planner/fields2cover/types/cells.hpp"

namespace autonomy::planning::planner::fields2cover::types {

Cell::Cell() {
  this->data_ = std::shared_ptr<OGRPolygon>(
    DownCast<OGRPolygon*>(OGRGeometryFactory::createGeometry(wkbPolygon)),
    [](OGRPolygon* f) {OGRGeometryFactory::destroyGeometry(f);});
}

Cell::Cell(const OGRGeometry* geom) {
  if (wkbFlatten(geom->getGeometryType()) == OGRwkbGeometryType::wkbPolygon) {
    this->data_ =
      std::shared_ptr<OGRPolygon>(DownCast<OGRPolygon*>(geom->clone()),
        [](OGRPolygon* f) {OGRGeometryFactory::destroyGeometry(f);});
  } else {
    throw std::invalid_argument(
        sstr("Cell(const OGRGeometry*): Type of OGRGeometry* is ",
        wkbFlatten(geom->getGeometryType()) , " instead of wkbPolygon(" ,
          OGRwkbGeometryType::wkbPolygon , ")"));
  }
}

Cell::Cell(const autonomy::planning::planner::fields2cover::types::LinearRing& ring) {
  this->AddRing(ring);
}

void Cell::GetGeometry(size_t i, LinearRing& ring) {
  if (i >= this->size()) {
    throw std::out_of_range(
        "Geometry does not contain point " + std::to_string(i));
  }
  ring = LinearRing(((i == 0) ?
    this->data_->getExteriorRing() :
    this->data_->getInteriorRing(i-1)), EmptyDestructor());
}

void Cell::GetGeometry(size_t i, LinearRing& ring) const {
  if (i >= this->size()) {
    throw std::out_of_range(
        "Geometry does not contain point " + std::to_string(i));
  }

  ring = LinearRing(((i == 0) ?
    this->data_->getExteriorRing() :
    this->data_->getInteriorRing(i-1)), EmptyDestructor());
}

LinearRing Cell::GetGeometry(size_t i) {
  if (i >= this->size()) {
    throw std::out_of_range(
        "Geometry does not contain point " + std::to_string(i));
  }
  return (i == 0) ? this->GetExteriorRing() : this->GetInteriorRing(i-1);
}

const LinearRing Cell::GetGeometry(size_t i) const {
  if (i >= this->size()) {
    throw std::out_of_range(
        "Geometry does not contain point " + std::to_string(i));
  }
  return (i == 0) ? this->GetExteriorRing() : this->GetInteriorRing(i-1);
}

void Cell::SetGeometry(size_t i, const LinearRing& line) {
  auto n = this->size();
  if (i < n) {
    Cell cell;
    for (size_t j = 0; j < n; ++j) {
      cell.AddGeometry((i == j) ? line : this->GetGeometry(j));
    }
    *this = cell;
    return;
  } else if (i != n) {
    for (size_t j = n; j < i; ++j) {
      this->AddGeometry(LinearRing());
    }
  }
  this->AddGeometry(line);
}

size_t Cell::size() const {
  return this->IsEmpty() ? 0 : (1 + this->data_->getNumInteriorRings());
}

void Cell::operator*=(double b) {
  for (auto&& ring : *this) {
    ring *= b;
  }
}

Cell Cell::buffer(const Cell& geom, double width) {
  return DestroyResGeom<Cell>(geom.OGRBuffer(width));
}

Cell Cell::buffer(const LineString& geom, double width) {
  return DestroyResGeom<Cell>(geom.OGRBuffer(width));
}

Cell Cell::buffer(const LinearRing& geom, double width) {
  Cell cell(geom);
  Cells b_cell_out {buffer(cell, fabs(width))};
  Cell b_cell_in = buffer(cell, -fabs(width));
  auto diff = b_cell_out.difference(b_cell_in);
  return (diff.size() > 0 ? diff.GetGeometry(0) : Cell());
}

Cell Cell::buffer(const Point& geom, double width) {
  return DestroyResGeom<Cell>(geom->Buffer(width));
}

Cell Cell::ConvexHull() const {
  return DestroyResGeom<Cell>(this->data_->ConvexHull());
}

void Cell::AddRing(const LinearRing& t) {
  auto r = t.Clone();
  this->data_->addRing(r.CloseRing().get());
}

void Cell::AddGeometry(const LinearRing& ring) {
  this->AddRing(ring);
}

const LinearRing Cell::GetExteriorRing() const {
  return LinearRing(this->data_->getExteriorRing());
}

const LinearRing Cell::GetInteriorRing(size_t i) const {
  return LinearRing(this->data_->getInteriorRing(i));
}


bool Cell::IsConvex() const {
  if (this->data_->getNumInteriorRings() > 0 || this->data_->get_Area() <= 0) {
    return false;
  }
  auto border = this->GetExteriorRing();
  if (border.size() < 2) {
    return false;
  }
  auto GetAng = [&border] (size_t i) {
    return Point::GetAngleFromPoints(
        border.GetGeometry(i),
        border.GetGeometry(i + 1),
        border.GetGeometry(i + 2));
  };
  bool inv = M_PI > GetAng(0);
  for (size_t i = 1; i < border.size() - 2; ++i) {
    if (inv != (GetAng(i) < M_PI)) {
      return false;
    }
  }
  return true;
}

LineString Cell::CreateSemiLongLine(const Point& point, double angle) const {
  return LineString({point,
    point.GetPointFromAngle(angle, this->GetMinSafeLength())});
}

LineString Cell::CreateStraightLongLine(
    const Point& point, double angle) const {
  return LineString({
    point.GetPointFromAngle(M_PI + angle, this->GetMinSafeLength()),
    point.GetPointFromAngle(angle,        this->GetMinSafeLength())});
}

MultiLineString Cell::GetLinesInside(const LineString& line) const {
  return MultiLineString::intersection(line, *this);
}

MultiLineString Cell::GetLinesInside(const MultiLineString& lines) const {
  return lines.intersection(*this);
}

bool Cell::IsPointInBorder(const Point& p) const {
  return p.touches(*this);
}

bool Cell::IsPointIn(const Point& p) const {
  return p.within(*this);
}

LineString Cell::CreateLineUntilBorder(
    const autonomy::planning::planner::fields2cover::types::Point& p, double ang) const {
  const MultiLineString ray(this->CreateSemiLongLine(p, ang));
  const auto intersections(ray.intersection(*this));
  Point best_point{p};
  double min_dist {std::numeric_limits<double>::max()};
  for (auto&& line : intersections) {
    for (auto&& p_l : line) {
      double dist = p_l.Distance(p);
      if (dist > 1e-5 && min_dist > dist) {
        best_point = p_l;
        min_dist = dist;
      }
    }
  }
  return (best_point == p) ? LineString() : LineString({p, best_point});
}

Point Cell::ClosestPointOnBorderTo(const Point& p) const {
  std::vector<double> dist;
  std::vector<Point> ps;
  for (auto&& ring : *this) {
    ps.emplace_back(ring.ClosestPointTo(p));
    dist.emplace_back(ps.back().Distance(p));
  }
  return ps[std::min_element(dist.begin(), dist.end()) - dist.begin()];
}


}  // namespace autonomy::planning::planner::fields2cover::types

