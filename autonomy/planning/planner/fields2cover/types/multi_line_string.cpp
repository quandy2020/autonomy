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
 * @file multi_line_string.cpp
 * @brief Fields2Cover MultiLineString implementation.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */



#include "autonomy/planning/planner/fields2cover/types/multi_line_string.hpp"

namespace autonomy::planning::planner::fields2cover::types {

MultiLineString::MultiLineString() {
  data_ = std::shared_ptr<NativeGeom>(static_cast<NativeGeom*>(
      GeomFactory::createGeometry(WkbKind::kMultiLineString)),
    [](NativeGeom* f) {GeomFactory::destroyGeometry(f);});
}

MultiLineString::MultiLineString(const NativeGeom* geom) {
  this->Append(geom);
}

MultiLineString::MultiLineString(const LineString& line) {
  this->AddGeometry(line);
}

MultiLineString::MultiLineString(const std::initializer_list<LineString>& ls) {
  for (auto&& line : ls) {
    this->AddGeometry(line);
  }
}

size_t MultiLineString::size() const {
  return IsEmpty() ? 0 : data_->getNumGeometries();
}

double MultiLineString::Length() const {
  return this->data_->get_Length();
}

void MultiLineString::operator*=(double b) {
  for (auto&& r : *this) {
    r *= b;
  }
}

void MultiLineString::Append(const NativeGeom* geom) {
  if (geom == nullptr) {
    return;
  }
  const WkbKind kind = wkbFlatten(geom->getGeometryType());
  if (kind == WkbKind::kLineString) {
    if (!geom->IsEmpty()) {
      data_->addGeometry(geom);
    }
  } else if (kind == WkbKind::kMultiLineString) {
    for (int i = 0; i < geom->getNumGeometries(); ++i) {
      data_->addGeometry(geom->getGeometryRef(i));
    }
  } else if (kind == WkbKind::kPolygon) {
    for (int i = 0; i < geom->getNumGeometries(); ++i) {
      data_->addGeometry(geom->getGeometryRef(i));
    }
  } else if (kind == WkbKind::kMultiPolygon) {
    for (int i = 0; i < geom->getNumGeometries(); ++i) {
      const NativeGeom* poly = geom->getGeometryRef(i);
      for (int j = 0; j < poly->getNumGeometries(); ++j) {
        data_->addGeometry(poly->getGeometryRef(j));
      }
    }
  }
}

void MultiLineString::GetGeometry(size_t i, LineString& line) {
  if (i >= this->size()) {
    throw std::out_of_range(
        "Geometry does not contain point " + std::to_string(i));
  }
  line = LineString(this->data_->getGeometryRef(i), EmptyDestructor());
}

void MultiLineString::GetGeometry(size_t i, LineString& line) const {
  if (i >= this->size()) {
    throw std::out_of_range(
        "Geometry does not contain point " + std::to_string(i));
  }
  line = LineString(this->data_->getGeometryRef(i), EmptyDestructor());
}

LineString MultiLineString::GetGeometry(size_t i) {
  if (i >= this->size()) {
    throw std::out_of_range(
        "Geometry does not contain point " + std::to_string(i));
  }
  return LineString(this->data_->getGeometryRef(i));
}

const LineString MultiLineString::GetGeometry(size_t i) const {
  if (i >= this->size()) {
    throw std::out_of_range(
        "Geometry does not contain point " + std::to_string(i));
  }
  return LineString(this->data_->getGeometryRef(i));
}

void MultiLineString::SetGeometry(size_t i, const LineString& line) {
  auto n = this->size();
  if (i < n) {
    MultiLineString lines;
    for (size_t j = 0; j < n; ++j) {
      lines.AddGeometry((i == j) ? line : this->GetGeometry(j));
    }
    *this = lines;
    return;
  } else if (i != n) {
    for (size_t j = n; j < i; ++j) {
      this->AddGeometry(LineString());
    }
  }
  this->AddGeometry(line);
}

void MultiLineString::AddGeometry(const LineString& line) {
  this->data_->addGeometry(line.get());
}

void MultiLineString::AddGeometry(const MultiLineString& lines) {
  for (auto&& line : lines) {
    AddGeometry(line);
  }
}

MultiLineString MultiLineString::GetLineSegments(const LineString& line) {
  MultiLineString res;
  for (size_t i = 0; i < line.size()-1; ++i) {
    res.AddGeometry(LineString({line.GetGeometry(i), line.GetGeometry(i+1)}));
  }
  return res;
}

MultiLineString MultiLineString::GetLineSegments(const LinearRing& line) {
  return GetLineSegments(LineString(line));
}

}  // namespace autonomy::planning::planner::fields2cover::types

