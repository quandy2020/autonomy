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
 * @file cells.cpp
 * @brief Fields2Cover Cells implementation.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */



#include <algorithm>
#include "autonomy/planning/planner/fields2cover/types/cells.hpp"

namespace autonomy::planning::planner::fields2cover::types {

Cells::Cells() {
  this->data_ = std::shared_ptr<NativeGeom>(
    DownCast<NativeGeom*>(
      GeomFactory::createGeometry(WkbKind::kMultiPolygon)),
    [](NativeGeom* f) {GeomFactory::destroyGeometry(f);});
}

Cells::Cells(const NativeGeom* geom) {
  if (wkbFlatten(geom->getGeometryType()) == WkbKind::kPolygon) {
    this->data_ = std::shared_ptr<NativeGeom>(DownCast<NativeGeom*>(
        GeomFactory::createGeometry(WkbKind::kMultiPolygon)),
        [](NativeGeom* f) {GeomFactory::destroyGeometry(f);});
    this->data_->addGeometry(geom->toPolygon());
  } else if (wkbFlatten(geom->getGeometryType()) ==
      WkbKind::kMultiPolygon) {
    this->data_ = std::shared_ptr<NativeGeom>(
        DownCast<NativeGeom*>(geom->clone()),
        [](NativeGeom* f) {GeomFactory::destroyGeometry(f);});
  } else if (wkbFlatten(geom->getGeometryType()) ==
      WkbKind::kGeometryCollection) {
    this->data_ = std::shared_ptr<NativeGeom>(
        DownCast<NativeGeom*>(
          GeomFactory::createGeometry(WkbKind::kMultiPolygon)),
            [](NativeGeom* f) {GeomFactory::destroyGeometry(f);});
    auto* gc = DownCast<const NativeGeom*>(geom);
    for (int i = 0; i < gc->getNumGeometries(); ++i) {
      auto* g_i = gc->getGeometryRef(i);
      if (wkbFlatten(g_i->getGeometryType()) ==
          WkbKind::kPolygon) {
        this->data_->addGeometry(g_i);
      }
    }
  } else {
    this->data_ = std::shared_ptr<NativeGeom>(
        DownCast<NativeGeom*>(
          GeomFactory::createGeometry(WkbKind::kMultiPolygon)),
            [](NativeGeom* f) {GeomFactory::destroyGeometry(f);});
  }
}


Cells::Cells(const Cell& c) {
  this->data_->addGeometry(c.get());
}

void Cells::operator*=(double b) {
  for (auto&& poly : *this) {
    poly *= b;
  }
}

void Cells::GetGeometry(size_t i, Cell& cell) {
  if (i >= this->size()) {
    throw std::out_of_range(
        "Geometry does not contain point " + std::to_string(i));
  }
  cell = Cell(this->data_->getGeometryRef(i), EmptyDestructor());
}

void Cells::GetGeometry(size_t i, Cell& cell) const {
  if (i >= this->size()) {
    throw std::out_of_range(
        "Geometry does not contain point " + std::to_string(i));
  }
  cell = Cell(this->data_->getGeometryRef(i), EmptyDestructor());
}

Cell Cells::GetGeometry(size_t i) {
  if (i >= this->size()) {
    throw std::out_of_range(
        "Geometry does not contain point " + std::to_string(i));
  }
  return Cell(this->data_->getGeometryRef(i));
}

const Cell Cells::GetGeometry(size_t i) const {
  if (i >= this->size()) {
    throw std::out_of_range(
        "Cells does not contain cell at " + std::to_string(i));
  }
  return Cell(this->data_->getGeometryRef(i));
}

void Cells::SetGeometry(size_t i, const Cell& cell) {
  auto n = this->size();
  if (i < n) {
    Cells cells;
    for (size_t j = 0; j < n; ++j) {
      cells.AddGeometry((i == j) ? cell : this->GetGeometry(j));
    }
    *this = cells;
    return;
  } else if (i != n) {
    for (size_t j = n; j < i; ++j) {
      this->AddGeometry(Cell());
    }
  }
  this->AddGeometry(cell);
}

const Cell Cells::GetCell(size_t i) const {
  return GetGeometry(i);
}

const LinearRing Cells::GetCellBorder(size_t i) const {
  // Check the index before asking OGR: GetGeometryRef() dereferences a null
  // collection entry when the index is out of range.
  if (i >= static_cast<size_t>(this->data_->getNumGeometries())) {
    throw std::out_of_range(
        "Cells does not contain cell at " + std::to_string(i));
  }
  return LinearRing(
      DownCast<NativeGeom*>(this->data_->getGeometryRef(i))->getExteriorRing());
}

const LinearRing Cells::GetInteriorRing(size_t i_cell, size_t i_ring) const {
  if (i_cell >= static_cast<size_t>(this->data_->getNumGeometries())) {
    throw std::out_of_range(
        "Cells does not contain cell at " + std::to_string(i_cell));
  }
  return LinearRing(DownCast<NativeGeom*>(this->data_->getGeometryRef(i_cell))
      ->getInteriorRing(i_ring));
}

void Cells::AddGeometry(const Cell& c) {
  this->data_->addGeometry(c.get());
}

void Cells::AddRing(size_t i, const LinearRing& ring) {
  if (i >= static_cast<size_t>(this->data_->getNumGeometries())) {
    throw std::out_of_range(
        "Cells does not contain cell at " + std::to_string(i));
  }
  DownCast<NativeGeom*>(this->data_->getGeometryRef(i))->addRing(
      ring.Clone().get());
}

size_t Cells::size() const {
  return IsEmpty() ? 0 : this->data_->getNumGeometries();
}

bool Cells::IsConvex() const {
  return (this->size() != 1) ? false :
    GetCell(0).IsConvex();
}

Cell Cells::ConvexHull() const {
  return DestroyResGeom<Cell>(this->data_->ConvexHull());
}

Cells Cells::intersection(const Cell& cell, const Cell& c) {
  return DestroyResGeom<Cells>(cell->Intersection(c.get()));
}

Cells Cells::intersection(const Cell& c) const {
  return DestroyResGeom<Cells>(c->Intersection(this->get()));
}

Cells Cells::intersection(const Cells& c) const {
  return DestroyResGeom<Cells>(this->data_->Intersection(c.get()));
}

Cells Cells::difference(const Cells& c) const {
  return DestroyResGeom<Cells>(this->data_->Difference(c.get()));
}

Cells Cells::difference(const Cell& c) const {
  return DestroyResGeom<Cells>(this->data_->Difference(c.get()));
}

Cells Cells::UnionOp(const Cells& c) const {
  return DestroyResGeom<Cells>(this->data_->Union(c.get()));
}

Cells Cells::UnionOp(const Cell& c) const {
  return DestroyResGeom<Cells>(this->data_->Union(c.get()));
}

Cells Cells::UnionCascaded() const {
  return DestroyResGeom<Cells>(this->data_->UnionCascaded());
}

namespace {
// The cut takes a 1e-8-wide strip out of the field, so each piece is
// reinflated by half that to keep the total size the same. Two chained
// cut-reinflate passes let GEOS collapse the second cut into a no-op, so
// every split line is buffered together into a single pass. A reinflated
// piece can also come out pinched into two, which Cell::buffer can't hold;
// collect into a fresh Cells instead.
template <class T, WkbKind R>
Cells SplitCellsByGeom(const Cells& self, const Geometry<T, R>& geom) {
  Cells raw = self.difference(self.buffer(geom, 1e-8));
  Cells cells;
  for (auto&& c : raw) {
    for (auto&& piece : Cells::buffer(c, 1e-8 * 0.5)) {
      cells.AddGeometry(piece);
    }
  }
  return cells;
}
}  // namespace

Cells Cells::SplitByLine(const LineString& line) const {
  return SplitCellsByGeom(*this, line);
}

Cells Cells::SplitByLine(const MultiLineString& lines) const {
  return SplitCellsByGeom(*this, lines);
}

LineString Cells::CreateSemiLongLine(const Point& point, double angle) const {
  return LineString({point,
    point.GetPointFromAngle(angle, this->GetMinSafeLength())});
}

LineString Cells::CreateStraightLongLine(
    const Point& point, double angle) const {
  return LineString({
    point.GetPointFromAngle(M_PI + angle, this->GetMinSafeLength()),
    point.GetPointFromAngle(angle,        this->GetMinSafeLength())});
}

MultiLineString Cells::GetLinesInside(const LineString& line) const {
  return MultiLineString::intersection(line, *this);
}

MultiLineString Cells::GetLinesInside(const MultiLineString& lines) const {
  return lines.intersection(*this);
}

Cells Cells::GetCellsInside(const Cells& cell) const {
  return this->intersection(cell);
}

bool Cells::IsPointInBorder(const Point& p) const {
  return p.touches(*this);
}

bool Cells::IsPointIn(const Point& p) const {
  return p.within(*this);
}

const Cell Cells::GetCellWherePoint(const Point& p) const {
  for (auto&& cell : *this) {
    if (p.touches(cell) || p.within(cell)) {
      return cell;
    }
  }
  return Cell();
}

const Cell Cells::GetCellWherePoint(const Point& p, double d_tol) const {
  for (auto&& cell : *this) {
    if (p.Distance(cell) <= d_tol) {
      return cell;
    }
  }
  return Cell();
}

LineString Cells::CreateLineUntilBorder(
    const autonomy::planning::planner::fields2cover::types::Point& p, double ang) const {
  return this->GetCellWherePoint(p).CreateLineUntilBorder(p, ang);
}

Cells Cells::buffer(double width) const {
  return DestroyResGeom<Cells>(this->BufferNative(width));
}

Point Cells::ClosestPointOnBorderTo(const Point& p) const {
  std::vector<double> dist;
  std::vector<Point> ps;
  for (auto&& cell : *this) {
    ps.emplace_back(cell.ClosestPointOnBorderTo(p));
    dist.emplace_back(ps.back().Distance(p));
  }
  return ps[std::min_element(dist.begin(), dist.end()) - dist.begin()];
}


}  // namespace autonomy::planning::planner::fields2cover::types

