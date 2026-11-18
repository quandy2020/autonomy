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
 * @file native_geom.cpp
 * @brief GDAL-free geometry kernel for Fields2Cover.
 */

#include "autonomy/planning/planner/fields2cover/types/native_geom.hpp"

#include <algorithm>
#include <type_traits>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <strings.h>
#include <utility>
#include <iterator>
#include <type_traits>

#include "autonomy/common/math/polygon2d.hpp"
#include "autonomy/common/math/vec2d.hpp"

namespace autonomy::planning::planner::fields2cover::types {
namespace {

using autonomy::common::math::Polygon2d;
using autonomy::common::math::Vec2d;

constexpr double kPi = 3.14159265358979323846;
constexpr double kEps = 1e-9;

bool NearlyEqual(double a, double b) { return std::fabs(a - b) < 1e-7; }

std::vector<Vec2d> ToVec2d(const std::vector<Coord>& coords) {
  std::vector<Vec2d> pts;
  pts.reserve(coords.size());
  for (const Coord& c : coords) {
    pts.emplace_back(c.x, c.y);
  }
  return pts;
}

std::vector<Coord> FromVec2d(const std::vector<Vec2d>& pts, double z = 0.0) {
  std::vector<Coord> coords;
  coords.reserve(pts.size());
  for (const Vec2d& p : pts) {
    coords.push_back({p.x(), p.y(), z});
  }
  return coords;
}

void CloseRing(std::vector<Coord>* coords) {
  if (coords->size() < 2) {
    return;
  }
  const Coord& a = coords->front();
  const Coord& b = coords->back();
  if (!NearlyEqual(a.x, b.x) || !NearlyEqual(a.y, b.y)) {
    coords->push_back(a);
  }
}


NativeGeom* NewKind(WkbKind kind) {
  auto* g = new NativeGeom();
  g->kind = kind;
  if (kind == WkbKind::kPoint) {
    g->coords.push_back({0.0, 0.0, 0.0});
  }
  return g;
}

NativeGeom* PolygonFromPoints(const std::vector<Vec2d>& pts) {
  auto* poly = NewKind(WkbKind::kPolygon);
  auto ring = std::make_shared<NativeGeom>();
  ring->kind = WkbKind::kLinearRing;
  ring->coords = FromVec2d(pts);
  CloseRing(&ring->coords);
  poly->parts.push_back(std::move(ring));
  return poly;
}

// Boost.Geometry preserves interior rings and disconnected components. Keep the
// conversion private so NativeGeom remains independent of the geometry backend.
namespace bg = boost::geometry;
using BGPoint = bg::model::d2::point_xy<double>;
using BGPolygon = bg::model::polygon<BGPoint>;
using BGPolygons = bg::model::multi_polygon<BGPolygon>;
using BGLine = bg::model::linestring<BGPoint>;
using BGLines = bg::model::multi_linestring<BGLine>;

bool IsPolygonKind(WkbKind kind) {
  return kind == WkbKind::kPolygon || kind == WkbKind::kMultiPolygon;
}

bool IsLineKind(WkbKind kind) {
  return kind == WkbKind::kLineString || kind == WkbKind::kLinearRing ||
         kind == WkbKind::kMultiLineString;
}

BGPolygon ToBoostPolygon(const NativeGeom& geom) {
  BGPolygon polygon;
  auto copy_ring = [](const std::vector<Coord>& coords, auto& ring) {
    for (const auto& c : coords) ring.emplace_back(c.x, c.y);
  };
  if (geom.kind == WkbKind::kPolygon && !geom.parts.empty()) {
    copy_ring(geom.parts.front()->coords, polygon.outer());
    for (size_t i = 1; i < geom.parts.size(); ++i) {
      polygon.inners().emplace_back();
      copy_ring(geom.parts[i]->coords, polygon.inners().back());
    }
  } else {
    copy_ring(geom.coords, polygon.outer());
  }
  bg::correct(polygon);
  return polygon;
}

BGPolygons ToBoostPolygons(const NativeGeom& geom) {
  BGPolygons result;
  if (geom.kind == WkbKind::kPolygon || geom.kind == WkbKind::kLinearRing) {
    auto polygon = ToBoostPolygon(geom);
    if (polygon.outer().size() >= 4) result.push_back(std::move(polygon));
  } else if (geom.kind == WkbKind::kMultiPolygon ||
             geom.kind == WkbKind::kGeometryCollection) {
    for (const auto& part : geom.parts) {
      if (!part) continue;
      auto polygons = ToBoostPolygons(*part);
      result.insert(result.end(), polygons.begin(), polygons.end());
    }
  }
  return result;
}

BGLines ToBoostLines(const NativeGeom& geom) {
  BGLines result;
  if (geom.kind == WkbKind::kMultiLineString) {
    for (const auto& part : geom.parts) {
      if (!part) continue;
      auto lines = ToBoostLines(*part);
      result.insert(result.end(), lines.begin(), lines.end());
    }
  } else if (geom.coords.size() >= 2) {
    BGLine line;
    for (const auto& c : geom.coords) line.emplace_back(c.x, c.y);
    result.push_back(std::move(line));
  }
  return result;
}

NativeGeom* FromBoostPolygons(const BGPolygons& polygons) {
  auto result = std::make_unique<NativeGeom>();
  result->kind = WkbKind::kMultiPolygon;
  for (const auto& polygon : polygons) {
    if (std::fabs(bg::area(polygon)) < kEps) continue;
    auto native = std::make_shared<NativeGeom>();
    native->kind = WkbKind::kPolygon;
    auto copy_ring = [&native](const auto& ring) {
      auto part = std::make_shared<NativeGeom>();
      part->kind = WkbKind::kLinearRing;
      for (const auto& p : ring)
        part->coords.push_back({bg::get<0>(p), bg::get<1>(p), 0.0});
      native->parts.push_back(std::move(part));
    };
    copy_ring(polygon.outer());
    for (const auto& ring : polygon.inners()) copy_ring(ring);
    result->parts.push_back(std::move(native));
  }
  // A Cell represents one polygon; Cells accepts either representation.
  if (result->parts.size() == 1) return result->parts.front()->clone();
  return result.release();
}

NativeGeom* FromBoostLines(const BGLines& lines) {
  auto result = std::make_unique<NativeGeom>();
  result->kind = WkbKind::kMultiLineString;
  for (const auto& line : lines) {
    if (bg::length(line) < kEps) continue;
    auto part = std::make_shared<NativeGeom>();
    part->kind = WkbKind::kLineString;
    for (const auto& p : line)
      part->coords.push_back({bg::get<0>(p), bg::get<1>(p), 0.0});
    result->parts.push_back(std::move(part));
  }
  return result.release();
}

// Buffering must retain narrow corners and exact neighboring strip boundaries.
// Boost's distance/1000 simplification and integer rescaling otherwise change
// near-collinear edges. Line buffers retain the backend rescaling policy.
template <class Distance>
struct PreciseBufferDistance : Distance {
  explicit PreciseBufferDistance(const Distance& distance) : Distance(distance) {}
  double simplify_distance() const { return 1e-12; }
};

template <class Geometry, class Distance>
NativeGeom* BoostBuffer(const Geometry& geometry, const Distance& distance) {
  BGPolygons output;
  bg::strategy::buffer::side_straight sides;
  bg::strategy::buffer::join_miter joins(5.0);
  bg::strategy::buffer::end_flat ends;
  bg::strategy::buffer::point_circle circles(32);
  if constexpr (std::is_same_v<Geometry, BGLines>) {
    bg::buffer(geometry, output, distance, sides, joins, ends, circles);
  } else if (!bg::is_empty(geometry)) {
    bg::strategy::intersection::cartesian_segments<> intersection;
    bg::detail::buffer::buffer_inserter<BGPolygon>(geometry,
        std::back_inserter(output), PreciseBufferDistance<Distance>(distance),
        sides, joins, ends, circles, intersection,
        bg::detail::no_rescale_policy());
  }
  return FromBoostPolygons(output);
}

double PointDistance(const Coord& a, const Coord& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double PolylineLength(const std::vector<Coord>& coords) {
  double len = 0.0;
  for (size_t i = 1; i < coords.size(); ++i) {
    len += PointDistance(coords[i - 1], coords[i]);
  }
  return len;
}

double RingArea(const std::vector<Coord>& coords) {
  if (coords.size() < 3) {
    return 0.0;
  }
  double area = 0.0;
  for (size_t i = 0; i + 1 < coords.size(); ++i) {
    area += coords[i].x * coords[i + 1].y - coords[i + 1].x * coords[i].y;
  }
  return 0.5 * area;
}

void CollectCoords(const NativeGeom& g, std::vector<Coord>* out) {
  if (!g.coords.empty() &&
      (g.kind == WkbKind::kPoint || g.kind == WkbKind::kLineString ||
       g.kind == WkbKind::kLinearRing)) {
    out->insert(out->end(), g.coords.begin(), g.coords.end());
  }
  for (const auto& part : g.parts) {
    if (part) {
      CollectCoords(*part, out);
    }
  }
}

std::string FormatCoords(const std::vector<Coord>& coords) {
  std::ostringstream oss;
  oss << std::setprecision(std::numeric_limits<double>::max_digits10);
  oss.setf(std::ios::fmtflags(0), std::ios::floatfield);
  for (size_t i = 0; i < coords.size(); ++i) {
    if (i > 0) {
      oss << ", ";
    }
    oss << coords[i].x << " " << coords[i].y;
    if (std::fabs(coords[i].z) > kEps) {
      oss << " " << coords[i].z;
    }
  }
  return oss.str();
}

std::string ToWkt(const NativeGeom& g) {
  switch (g.kind) {
    case WkbKind::kPoint:
      if (g.coords.empty()) {
        return "POINT EMPTY";
      }
      return "POINT (" + FormatCoords(g.coords) + ")";
    case WkbKind::kLineString:
      return "LINESTRING (" + FormatCoords(g.coords) + ")";
    case WkbKind::kLinearRing:
      return "LINEARRING (" + FormatCoords(g.coords) + ")";
    case WkbKind::kPolygon: {
      std::ostringstream oss;
      oss << "POLYGON (";
      for (size_t i = 0; i < g.parts.size(); ++i) {
        if (i > 0) {
          oss << ", ";
        }
        oss << "(" << FormatCoords(g.parts[i]->coords) << ")";
      }
      oss << ")";
      return oss.str();
    }
    case WkbKind::kMultiPoint: {
      std::ostringstream oss;
      oss << "MULTIPOINT (";
      for (size_t i = 0; i < g.parts.size(); ++i) {
        if (i > 0) {
          oss << ", ";
        }
        oss << "(" << FormatCoords(g.parts[i]->coords) << ")";
      }
      oss << ")";
      return oss.str();
    }
    case WkbKind::kMultiLineString: {
      std::ostringstream oss;
      oss << "MULTILINESTRING (";
      for (size_t i = 0; i < g.parts.size(); ++i) {
        if (i > 0) {
          oss << ", ";
        }
        oss << "(" << FormatCoords(g.parts[i]->coords) << ")";
      }
      oss << ")";
      return oss.str();
    }
    case WkbKind::kMultiPolygon: {
      std::ostringstream oss;
      oss << "MULTIPOLYGON (";
      for (size_t i = 0; i < g.parts.size(); ++i) {
        if (i > 0) {
          oss << ", ";
        }
        oss << "(";
        for (size_t r = 0; r < g.parts[i]->parts.size(); ++r) {
          if (r > 0) {
            oss << ", ";
          }
          oss << "(" << FormatCoords(g.parts[i]->parts[r]->coords) << ")";
        }
        oss << ")";
      }
      oss << ")";
      return oss.str();
    }
    default:
      return "GEOMETRYCOLLECTION EMPTY";
  }
}

void SkipSpace(const char** p) {
  while (**p && std::isspace(static_cast<unsigned char>(**p))) {
    ++(*p);
  }
}

bool Consume(const char** p, const char* token) {
  SkipSpace(p);
  const size_t n = std::strlen(token);
  if (::strncasecmp(*p, token, n) == 0) {
    *p += n;
    return true;
  }
  return false;
}

std::vector<Coord> ParseCoordList(const char** p) {
  std::vector<Coord> coords;
  SkipSpace(p);
  if (**p == '(') {
    ++(*p);
  }
  while (**p && **p != ')') {
    SkipSpace(p);
    if (**p == ',' || **p == '(' || **p == ')') {
      if (**p == ',') {
        ++(*p);
      }
      continue;
    }
    Coord c;
    char* end = nullptr;
    c.x = std::strtod(*p, &end);
    *p = end;
    c.y = std::strtod(*p, &end);
    *p = end;
    SkipSpace(p);
    if (**p && **p != ',' && **p != ')') {
      c.z = std::strtod(*p, &end);
      *p = end;
    }
    coords.push_back(c);
    SkipSpace(p);
    if (**p == ',') {
      ++(*p);
    }
  }
  if (**p == ')') {
    ++(*p);
  }
  return coords;
}

NativeGeom* ParseWkt(const char** p);

NativeGeom* ParseRingList(const char** p, WkbKind kind) {
  auto* g = NewKind(kind);
  SkipSpace(p);
  if (**p == '(') {
    ++(*p);
  }
  while (**p && **p != ')') {
    SkipSpace(p);
    if (**p == '(') {
      auto ring = std::shared_ptr<NativeGeom>(NewKind(WkbKind::kLinearRing));
      ring->coords = ParseCoordList(p);
      g->parts.push_back(std::move(ring));
    } else if (**p == ',') {
      ++(*p);
    } else {
      break;
    }
    SkipSpace(p);
  }
  if (**p == ')') {
    ++(*p);
  }
  return g;
}

NativeGeom* ParseWkt(const char** p) {
  SkipSpace(p);
  if (Consume(p, "POINT")) {
    auto* g = NewKind(WkbKind::kPoint);
    if (Consume(p, "EMPTY")) {
      g->coords.clear();
      return g;
    }
    g->coords = ParseCoordList(p);
    return g;
  }
  if (Consume(p, "LINESTRING")) {
    auto* g = NewKind(WkbKind::kLineString);
    g->coords = ParseCoordList(p);
    return g;
  }
  if (Consume(p, "LINEARRING")) {
    auto* g = NewKind(WkbKind::kLinearRing);
    g->coords = ParseCoordList(p);
    return g;
  }
  if (Consume(p, "POLYGON")) {
    return ParseRingList(p, WkbKind::kPolygon);
  }
  if (Consume(p, "MULTIPOINT")) {
    auto* g = ParseRingList(p, WkbKind::kMultiPoint);
    for (auto& part : g->parts) {
      part->kind = WkbKind::kPoint;
    }
    return g;
  }
  if (Consume(p, "MULTILINESTRING")) {
    auto* g = ParseRingList(p, WkbKind::kMultiLineString);
    for (auto& part : g->parts) {
      part->kind = WkbKind::kLineString;
    }
    return g;
  }
  if (Consume(p, "MULTIPOLYGON")) {
    auto* g = NewKind(WkbKind::kMultiPolygon);
    SkipSpace(p);
    if (**p == '(') {
      ++(*p);
    }
    while (**p && **p != ')') {
      SkipSpace(p);
      if (**p == '(') {
        NativeGeom* poly = ParseRingList(p, WkbKind::kPolygon);
        g->parts.emplace_back(poly);
      } else if (**p == ',') {
        ++(*p);
      } else {
        break;
      }
    }
    if (**p == ')') {
      ++(*p);
    }
    return g;
  }
  return NewKind(WkbKind::kUnknown);
}

char* DupCString(const std::string& s) {
  char* out = static_cast<char*>(std::malloc(s.size() + 1));
  if (out == nullptr) {
    throw std::bad_alloc();
  }
  std::memcpy(out, s.c_str(), s.size() + 1);
  return out;
}

NativeGeom* BufferPoint(const Coord& c, double dist) {
  constexpr int kSegs = 16;
  std::vector<Vec2d> pts;
  pts.reserve(kSegs);
  for (int i = 0; i < kSegs; ++i) {
    const double a = 2.0 * kPi * static_cast<double>(i) / kSegs;
    pts.emplace_back(c.x + dist * std::cos(a), c.y + dist * std::sin(a));
  }
  return PolygonFromPoints(pts);
}

NativeGeom* BufferPolygon(const NativeGeom& geom, double dist) {
  if (!std::isfinite(dist)) throw std::invalid_argument("non-finite buffer distance");
  return BoostBuffer(ToBoostPolygons(geom),
                     bg::strategy::buffer::distance_symmetric<double>(dist));
}

NativeGeom* ClipLineByPolygon(const NativeGeom& line, const NativeGeom& poly) {
  BGLines clipped;
  bg::intersection(ToBoostLines(line), ToBoostPolygons(poly), clipped);
  return FromBoostLines(clipped);
}

}  // namespace

NativeGeom::NativeGeom(double x, double y, double z) : kind(WkbKind::kPoint) {
  coords.push_back({x, y, z});
}

NativeGeom* NativeGeom::clone() const {
  auto* g = new NativeGeom();
  g->kind = kind;
  g->coords = coords;
  g->parts.reserve(parts.size());
  for (const auto& part : parts) {
    g->parts.emplace_back(part ? part->clone() : nullptr);
  }
  return g;
}

bool NativeGeom::IsEmpty() const {
  if (!coords.empty()) {
    return false;
  }
  for (const auto& part : parts) {
    if (part && !part->IsEmpty()) {
      return false;
    }
  }
  return true;
}

bool NativeGeom::Equals(const NativeGeom* other) const {
  if (other == nullptr || kind != other->kind) {
    return false;
  }
  if (coords.size() != other->coords.size() ||
      parts.size() != other->parts.size()) {
    return false;
  }
  for (size_t i = 0; i < coords.size(); ++i) {
    if (!NearlyEqual(coords[i].x, other->coords[i].x) ||
        !NearlyEqual(coords[i].y, other->coords[i].y) ||
        !NearlyEqual(coords[i].z, other->coords[i].z)) {
      return false;
    }
  }
  for (size_t i = 0; i < parts.size(); ++i) {
    if (!parts[i]->Equals(other->parts[i].get())) {
      return false;
    }
  }
  return true;
}

void NativeGeom::getEnvelope(Envelope* envelope) const {
  std::vector<Coord> all;
  CollectCoords(*this, &all);
  if (all.empty()) {
    *envelope = Envelope{};
    return;
  }
  envelope->MinX = envelope->MaxX = all[0].x;
  envelope->MinY = envelope->MaxY = all[0].y;
  for (const Coord& c : all) {
    envelope->MinX = std::min(envelope->MinX, c.x);
    envelope->MaxX = std::max(envelope->MaxX, c.x);
    envelope->MinY = std::min(envelope->MinY, c.y);
    envelope->MaxY = std::max(envelope->MaxY, c.y);
  }
}

double NativeGeom::getX() const { return coords.empty() ? 0.0 : coords[0].x; }
double NativeGeom::getY() const { return coords.empty() ? 0.0 : coords[0].y; }
double NativeGeom::getZ() const { return coords.empty() ? 0.0 : coords[0].z; }
double NativeGeom::getX(int i) const { return coords[static_cast<size_t>(i)].x; }
double NativeGeom::getY(int i) const { return coords[static_cast<size_t>(i)].y; }
double NativeGeom::getZ(int i) const { return coords[static_cast<size_t>(i)].z; }

void NativeGeom::setX(double x) {
  if (coords.empty()) {
    coords.push_back({});
  }
  coords[0].x = x;
}
void NativeGeom::setY(double y) {
  if (coords.empty()) {
    coords.push_back({});
  }
  coords[0].y = y;
}
void NativeGeom::setZ(double z) {
  if (coords.empty()) {
    coords.push_back({});
  }
  coords[0].z = z;
}

int NativeGeom::getNumPoints() const { return static_cast<int>(coords.size()); }

void NativeGeom::addPoint(double x, double y, double z) {
  coords.push_back({x, y, z});
}

void NativeGeom::setPoint(int i, double x, double y, double z) {
  const size_t idx = static_cast<size_t>(i);
  if (idx >= coords.size()) {
    coords.resize(idx + 1);
  }
  coords[idx] = {x, y, z};
}

void NativeGeom::getPoint(int i, NativeGeom* out) const {
  const Coord& c = coords[static_cast<size_t>(i)];
  out->kind = WkbKind::kPoint;
  out->coords = {c};
  out->parts.clear();
}

void NativeGeom::reversePoints() {
  std::reverse(coords.begin(), coords.end());
}

void NativeGeom::removePoint(int i) {
  if (i >= 0 && static_cast<size_t>(i) < coords.size()) {
    coords.erase(coords.begin() + i);
  }
}

int NativeGeom::getNumGeometries() const {
  return static_cast<int>(parts.size());
}

NativeGeom* NativeGeom::getGeometryRef(int i) { return parts[i].get(); }
const NativeGeom* NativeGeom::getGeometryRef(int i) const {
  return parts[i].get();
}

void NativeGeom::addGeometry(const NativeGeom* geom) {
  if (geom == nullptr) {
    return;
  }
  parts.emplace_back(geom->clone());
}

NativeGeom* NativeGeom::getExteriorRing() {
  return parts.empty() ? nullptr : parts[0].get();
}
const NativeGeom* NativeGeom::getExteriorRing() const {
  return parts.empty() ? nullptr : parts[0].get();
}
NativeGeom* NativeGeom::getInteriorRing(int i) {
  return parts[static_cast<size_t>(i) + 1].get();
}
const NativeGeom* NativeGeom::getInteriorRing(int i) const {
  return parts[static_cast<size_t>(i) + 1].get();
}
int NativeGeom::getNumInteriorRings() const {
  return parts.empty() ? 0 : static_cast<int>(parts.size()) - 1;
}

void NativeGeom::addRing(const NativeGeom* ring) {
  if (ring == nullptr) {
    return;
  }
  auto owned = std::shared_ptr<NativeGeom>(ring->clone());
  owned->kind = WkbKind::kLinearRing;
  CloseRing(&owned->coords);
  parts.push_back(std::move(owned));
}

double NativeGeom::get_Length() const {
  if (kind == WkbKind::kLineString || kind == WkbKind::kLinearRing) {
    return PolylineLength(coords);
  }
  double len = 0.0;
  for (const auto& part : parts) {
    if (part) {
      len += part->get_Length();
    }
  }
  return len;
}

double NativeGeom::get_Area() const {
  if (kind == WkbKind::kLinearRing || kind == WkbKind::kLineString) {
    return std::fabs(RingArea(coords));
  }
  if (kind == WkbKind::kPolygon) {
    if (parts.empty()) {
      return 0.0;
    }
    double area = std::fabs(RingArea(parts[0]->coords));
    for (size_t i = 1; i < parts.size(); ++i) {
      area -= std::fabs(RingArea(parts[i]->coords));
    }
    return std::max(0.0, area);
  }
  double area = 0.0;
  for (const auto& part : parts) {
    if (part) {
      area += part->get_Area();
    }
  }
  return area;
}

bool NativeGeom::isClockwise() const {
  return RingArea(coords) < 0.0;
}

bool NativeGeom::get_IsClosed() const {
  if (coords.size() < 2) {
    return false;
  }
  return NearlyEqual(coords.front().x, coords.back().x) &&
         NearlyEqual(coords.front().y, coords.back().y);
}

double NativeGeom::Distance(const NativeGeom* other) const {
  if (other == nullptr || IsEmpty() || other->IsEmpty()) {
    return std::numeric_limits<double>::infinity();
  }
  if (kind == WkbKind::kPoint && other->kind == WkbKind::kPoint) {
    return PointDistance(coords.front(), other->coords.front());
  }
  if (kind == WkbKind::kPoint) {
    const BGPoint point(getX(), getY());
    if (IsPolygonKind(other->kind))
      return bg::distance(point, ToBoostPolygons(*other));
    if (IsLineKind(other->kind))
      return bg::distance(point, ToBoostLines(*other));
  }
  if (other->kind == WkbKind::kPoint) return other->Distance(this);
  if (IsPolygonKind(kind)) {
    if (IsPolygonKind(other->kind))
      return bg::distance(ToBoostPolygons(*this), ToBoostPolygons(*other));
    if (IsLineKind(other->kind))
      return bg::distance(ToBoostLines(*other), ToBoostPolygons(*this));
  }
  if (IsLineKind(kind)) {
    if (IsPolygonKind(other->kind)) return other->Distance(this);
    if (IsLineKind(other->kind))
      return bg::distance(ToBoostLines(*this), ToBoostLines(*other));
  }
  // Collections contain geometries, not a polygon through their vertices.
  double distance = std::numeric_limits<double>::infinity();
  for (const auto& part : parts)
    if (part) distance = std::min(distance, part->Distance(other));
  for (const auto& part : other->parts)
    if (part) distance = std::min(distance, Distance(part.get()));
  return distance;
}

bool NativeGeom::Intersects(const NativeGeom* other) const {
  if (other == nullptr || IsEmpty() || other->IsEmpty()) return false;
  if (IsPolygonKind(kind)) {
    const auto polygons = ToBoostPolygons(*this);
    if (IsPolygonKind(other->kind))
      return bg::intersects(polygons, ToBoostPolygons(*other));
    if (IsLineKind(other->kind))
      return bg::intersects(polygons, ToBoostLines(*other));
    if (other->kind == WkbKind::kPoint)
      return bg::covered_by(BGPoint(other->getX(), other->getY()), polygons);
  }
  if (IsPolygonKind(other->kind)) return other->Intersects(this);
  if (IsLineKind(kind) && IsLineKind(other->kind))
    return bg::intersects(ToBoostLines(*this), ToBoostLines(*other));
  return Distance(other) < 1e-6;
}

bool NativeGeom::Disjoint(const NativeGeom* other) const {
  return !Intersects(other);
}

bool NativeGeom::Contains(const NativeGeom* other) const {
  if (other == nullptr || IsEmpty() || other->IsEmpty() || !IsPolygonKind(kind))
    return false;
  const auto polygons = ToBoostPolygons(*this);
  if (other->kind == WkbKind::kPoint)
    return bg::covered_by(BGPoint(other->getX(), other->getY()), polygons);
  if (IsPolygonKind(other->kind)) {
    BGPolygons outside;
    bg::difference(ToBoostPolygons(*other), polygons, outside);
    return outside.empty() || std::fabs(bg::area(outside)) < kEps;
  }
  if (IsLineKind(other->kind)) {
    BGLines outside;
    bg::difference(ToBoostLines(*other), polygons, outside);
    return bg::length(outside) < kEps;
  }
  return false;
}

bool NativeGeom::Within(const NativeGeom* other) const {
  return other != nullptr && other->Contains(this);
}

bool NativeGeom::Touches(const NativeGeom* other) const {
  if (other == nullptr || IsEmpty() || other->IsEmpty()) return false;
  if (kind == WkbKind::kPoint && IsPolygonKind(other->kind)) {
    const BGPoint point(getX(), getY());
    const auto polygons = ToBoostPolygons(*other);
    return bg::covered_by(point, polygons) && !bg::within(point, polygons);
  }
  if (other->kind == WkbKind::kPoint && IsPolygonKind(kind)) return other->Touches(this);
  if (IsPolygonKind(kind) && IsPolygonKind(other->kind))
    return bg::touches(ToBoostPolygons(*this), ToBoostPolygons(*other));
  return Intersects(other) && !Contains(other) && !other->Contains(this);
}

bool NativeGeom::Crosses(const NativeGeom* other) const {
  return Intersects(other) && !Contains(other) && !Within(other);
}

NativeGeom* NativeGeom::Intersection(const NativeGeom* other) const {
  if (other == nullptr) return NewKind(WkbKind::kMultiPolygon);
  if (IsLineKind(kind) && IsPolygonKind(other->kind))
    return ClipLineByPolygon(*this, *other);
  if (IsPolygonKind(kind) && IsLineKind(other->kind))
    return ClipLineByPolygon(*other, *this);
  if (IsPolygonKind(kind) && IsPolygonKind(other->kind)) {
    BGPolygons result;
    bg::intersection(ToBoostPolygons(*this), ToBoostPolygons(*other), result);
    return FromBoostPolygons(result);
  }
  if (Contains(other)) return other->clone();
  if (Within(other)) return clone();
  return NewKind(WkbKind::kMultiPolygon);
}

NativeGeom* NativeGeom::Difference(const NativeGeom* other) const {
  if (other == nullptr || other->IsEmpty()) return clone();
  if (IsPolygonKind(kind) && IsPolygonKind(other->kind)) {
    BGPolygons result;
    bg::difference(ToBoostPolygons(*this), ToBoostPolygons(*other), result);
    return FromBoostPolygons(result);
  }
  if (IsLineKind(kind) && IsPolygonKind(other->kind)) {
    BGLines result;
    bg::difference(ToBoostLines(*this), ToBoostPolygons(*other), result);
    return FromBoostLines(result);
  }
  if (Within(other)) return NewKind(kind);
  return clone();
}

NativeGeom* NativeGeom::Union(const NativeGeom* other) const {
  if (other == nullptr || other->IsEmpty()) return clone();
  if (IsEmpty()) return other->clone();
  if (IsPolygonKind(kind) && IsPolygonKind(other->kind)) {
    BGPolygons result;
    bg::union_(ToBoostPolygons(*this), ToBoostPolygons(*other), result);
    return FromBoostPolygons(result);
  }
  auto result = std::make_unique<NativeGeom>();
  result->kind = WkbKind::kGeometryCollection;
  result->addGeometry(this);
  result->addGeometry(other);
  return result.release();
}

NativeGeom* NativeGeom::UnionCascaded() const {
  if (parts.empty()) {
    return clone();
  }
  NativeGeom* acc = parts.front()->clone();
  for (size_t i = 1; i < parts.size(); ++i) {
    NativeGeom* next = acc->Union(parts[i].get());
    delete acc;
    acc = next;
  }
  return acc;
}

NativeGeom* NativeGeom::ConvexHull() const {
  std::vector<Coord> all;
  CollectCoords(*this, &all);
  std::vector<Vec2d> pts = ToVec2d(all);
  Polygon2d hull;
  if (!Polygon2d::ComputeConvexHull(pts, &hull) || hull.num_points() < 3) {
    return NewKind(WkbKind::kPolygon);
  }
  return PolygonFromPoints(hull.points());
}

NativeGeom* NativeGeom::Buffer(double dist, int side) const {
  if (!std::isfinite(dist)) throw std::invalid_argument("non-finite buffer distance");
  if (kind == WkbKind::kPoint)
    return BufferPoint(coords.empty() ? Coord{} : coords.front(), std::fabs(dist));
  if (IsPolygonKind(kind) || (kind == WkbKind::kLinearRing && dist < 0.0))
    return BufferPolygon(*this, dist);
  if (IsLineKind(kind)) {
    const auto lines = ToBoostLines(*this);
    if (side != 0)
      return BoostBuffer(lines, bg::strategy::buffer::distance_asymmetric<double>(
          side > 0 ? std::fabs(dist) : 0.0, side < 0 ? std::fabs(dist) : 0.0));
    return BoostBuffer(lines, bg::strategy::buffer::distance_symmetric<double>(std::fabs(dist)));
  }
  return NewKind(WkbKind::kMultiPolygon);
}

NativeGeom* NativeGeom::SimplifyPreserveTopology(double d_tol) const {
  NativeGeom* g = clone();
  if (g->coords.size() <= 2) {
    return g;
  }
  std::vector<Coord> simplified;
  simplified.push_back(g->coords.front());
  for (size_t i = 1; i < g->coords.size(); ++i) {
    if (PointDistance(simplified.back(), g->coords[i]) >= d_tol) {
      simplified.push_back(g->coords[i]);
    }
  }
  if (g->get_IsClosed() && simplified.size() >= 2) {
    CloseRing(&simplified);
  }
  g->coords = std::move(simplified);
  return g;
}

void NativeGeom::exportToWkt(char** wkt) const {
  *wkt = DupCString(ToWkt(*this));
}

void NativeGeom::importFromWkt(const char** wkt) {
  NativeGeom* parsed = ParseWkt(wkt);
  *this = *parsed;
  delete parsed;
}

char* NativeGeom::exportToGML() const { return DupCString(ToWkt(*this)); }
char* NativeGeom::exportToKML() const { return DupCString(ToWkt(*this)); }
char* NativeGeom::exportToJson() const { return DupCString(ToWkt(*this)); }

void NativeGeom::transform(const CoordTransform* ct) {
  if (ct == nullptr) {
    return;
  }
  for (Coord& c : coords) {
    ct->Transform(&c.x, &c.y, &c.z);
  }
  for (auto& part : parts) {
    if (part) {
      part->transform(ct);
    }
  }
}

NativeGeom* GeomFactory::createGeometry(WkbKind kind) { return NewKind(kind); }

NativeGeom* GeomFactory::createFromWkt(const std::string& wkt) {
  const char* p = wkt.c_str();
  return ParseWkt(&p);
}

void GeomFactory::destroyGeometry(NativeGeom* geom) { delete geom; }

namespace {

std::string Upper(std::string s) {
  for (char& c : s) {
    c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  }
  return s;
}

int ParseIntAfter(const std::string& s, const std::string& key) {
  const size_t pos = Upper(s).find(Upper(key));
  if (pos == std::string::npos) {
    return 0;
  }
  size_t i = pos + key.size();
  while (i < s.size() && !std::isdigit(static_cast<unsigned char>(s[i]))) {
    ++i;
  }
  return std::atoi(s.c_str() + i);
}

}  // namespace

CoordTransform::Spec CoordTransform::Parse(const std::string& crs) {
  Spec spec;
  const std::string u = Upper(crs);
  if (u.empty() || u == "LOCAL") {
    spec.frame = Frame::kIdentity;
    return spec;
  }
  if (u.find("UTM") != std::string::npos) {
    spec.frame = Frame::kUtm;
    spec.zone = ParseIntAfter(crs, "UTM:");
    if (spec.zone == 0) {
      spec.zone = ParseIntAfter(crs, "ZONE=");
    }
    spec.north = true;
    const size_t p = u.find("UTM:");
    if (p != std::string::npos) {
      size_t i = p + 4;
      while (i < u.size() && std::isdigit(static_cast<unsigned char>(u[i]))) {
        ++i;
      }
      if (i < u.size() && u[i] == 'S') {
        spec.north = false;
      }
    }
    return spec;
  }
  if (u.find("EPSG:4326") != std::string::npos ||
      u.find("EPSG:4258") != std::string::npos) {
    spec.frame = Frame::kLonLat;
    return spec;
  }
  spec.frame = Frame::kIdentity;
  return spec;
}

CoordTransform::CoordTransform(const std::string& from_crs,
                               const std::string& to_crs)
    : from_(Parse(from_crs)), to_(Parse(to_crs)) {}

void CoordTransform::LonLatToUtm(double lon, double lat, int zone, bool north,
                                 double* easting, double* northing) {
  const double a = 6378137.0;
  const double f = 1.0 / 298.257223563;
  const double k0 = 0.9996;
  const double e2 = f * (2.0 - f);
  const double e_p2 = e2 / (1.0 - e2);
  const double lon0 = (zone * 6 - 183) * kPi / 180.0;
  const double phi = lat * kPi / 180.0;
  const double lam = lon * kPi / 180.0 - lon0;
  const double sin_phi = std::sin(phi);
  const double cos_phi = std::cos(phi);
  const double tan_phi = std::tan(phi);
  const double n = a / std::sqrt(1.0 - e2 * sin_phi * sin_phi);
  const double t = tan_phi * tan_phi;
  const double c = e_p2 * cos_phi * cos_phi;
  const double a_term = lam * cos_phi;
  const double m =
      a *
      ((1.0 - e2 / 4.0 - 3.0 * e2 * e2 / 64.0 - 5.0 * e2 * e2 * e2 / 256.0) *
           phi -
       (3.0 * e2 / 8.0 + 3.0 * e2 * e2 / 32.0 + 45.0 * e2 * e2 * e2 / 1024.0) *
           std::sin(2.0 * phi) +
       (15.0 * e2 * e2 / 256.0 + 45.0 * e2 * e2 * e2 / 1024.0) *
           std::sin(4.0 * phi) -
       (35.0 * e2 * e2 * e2 / 3072.0) * std::sin(6.0 * phi));
  *easting =
      k0 * n *
          (a_term + (1.0 - t + c) * std::pow(a_term, 3) / 6.0 +
           (5.0 - 18.0 * t + t * t + 72.0 * c - 58.0 * e_p2) *
               std::pow(a_term, 5) / 120.0) +
      500000.0;
  *northing =
      k0 *
      (m +
       n * tan_phi *
           (a_term * a_term / 2.0 +
            (5.0 - t + 9.0 * c + 4.0 * c * c) * std::pow(a_term, 4) / 24.0 +
            (61.0 - 58.0 * t + t * t + 600.0 * c - 330.0 * e_p2) *
                std::pow(a_term, 6) / 720.0));
  if (!north) {
    *northing += 10000000.0;
  }
}

void CoordTransform::UtmToLonLat(double easting, double northing, int zone,
                                 bool north, double* lon, double* lat) {
  const double a = 6378137.0;
  const double f = 1.0 / 298.257223563;
  const double k0 = 0.9996;
  const double e2 = f * (2.0 - f);
  const double e1 = (1.0 - std::sqrt(1.0 - e2)) / (1.0 + std::sqrt(1.0 - e2));
  const double x = easting - 500000.0;
  double y = northing;
  if (!north) {
    y -= 10000000.0;
  }
  const double m = y / k0;
  const double mu =
      m /
      (a * (1.0 - e2 / 4.0 - 3.0 * e2 * e2 / 64.0 - 5.0 * e2 * e2 * e2 / 256.0));
  const double phi1 =
      mu + (3.0 * e1 / 2.0 - 27.0 * std::pow(e1, 3) / 32.0) * std::sin(2.0 * mu) +
      (21.0 * e1 * e1 / 16.0 - 55.0 * std::pow(e1, 4) / 32.0) *
          std::sin(4.0 * mu) +
      (151.0 * std::pow(e1, 3) / 96.0) * std::sin(6.0 * mu);
  const double sin_phi = std::sin(phi1);
  const double cos_phi = std::cos(phi1);
  const double tan_phi = std::tan(phi1);
  const double e_p2 = e2 / (1.0 - e2);
  const double n = a / std::sqrt(1.0 - e2 * sin_phi * sin_phi);
  const double t = tan_phi * tan_phi;
  const double c = e_p2 * cos_phi * cos_phi;
  const double r =
      a * (1.0 - e2) / std::pow(1.0 - e2 * sin_phi * sin_phi, 1.5);
  const double d = x / (n * k0);
  *lat =
      (phi1 -
       (n * tan_phi / r) *
           (d * d / 2.0 -
            (5.0 + 3.0 * t + 10.0 * c - 4.0 * c * c - 9.0 * e_p2) *
                std::pow(d, 4) / 24.0 +
            (61.0 + 90.0 * t + 298.0 * c + 45.0 * t * t - 252.0 * e_p2 -
             3.0 * c * c) *
                std::pow(d, 6) / 720.0)) *
      180.0 / kPi;
  const double lon0 = zone * 6 - 183;
  *lon =
      lon0 +
      ((d - (1.0 + 2.0 * t + c) * std::pow(d, 3) / 6.0 +
        (5.0 - 2.0 * c + 28.0 * t - 3.0 * c * c + 8.0 * e_p2 + 24.0 * t * t) *
            std::pow(d, 5) / 120.0) /
       cos_phi) *
          180.0 / kPi;
}

bool CoordTransform::Transform(double* x, double* y, double* /*z*/) const {
  if (from_.frame == to_.frame &&
      (from_.frame != Frame::kUtm ||
       (from_.zone == to_.zone && from_.north == to_.north))) {
    return true;
  }
  double lon = *x;
  double lat = *y;
  if (from_.frame == Frame::kUtm) {
    UtmToLonLat(*x, *y, from_.zone, from_.north, &lon, &lat);
  }
  if (to_.frame == Frame::kUtm) {
    int zone = to_.zone;
    if (zone <= 0) {
      zone = static_cast<int>(std::floor((lon + 180.0) / 6.0)) + 1;
    }
    LonLatToUtm(lon, lat, zone, to_.north, x, y);
    return true;
  }
  if (to_.frame == Frame::kLonLat) {
    *x = lon;
    *y = lat;
    return true;
  }
  return true;
}

}  // namespace autonomy::planning::planner::fields2cover::types
