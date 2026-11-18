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

#pragma once

/**
 * @file native_geom.hpp
 * @brief GDAL-free geometry kernel for Fields2Cover, built on common/math.
 */

#include <memory>
#include <string>
#include <vector>

namespace autonomy::planning::planner::fields2cover::types {

/**
 * @enum autonomy::planning::planner::fields2cover::types::WkbKind
 * @brief Geometry kind used by the native Fields2Cover kernel.
 */
enum class WkbKind {
  kUnknown = 0,
  kPoint = 1,
  kLineString = 2,
  kPolygon = 3,
  kMultiPoint = 4,
  kMultiLineString = 5,
  kMultiPolygon = 6,
  kGeometryCollection = 7,
  kLinearRing = 101,
};

/**
 * @class autonomy::planning::planner::fields2cover::types::Envelope
 * @brief Axis-aligned bounds of a geometry.
 */
struct Envelope {
  double MinX{0.0};
  double MaxX{0.0};
  double MinY{0.0};
  double MaxY{0.0};
};

/**
 * @class autonomy::planning::planner::fields2cover::types::Coord
 * @brief 3D vertex used by the native geometry kernel.
 */
struct Coord {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

/**
 * @class autonomy::planning::planner::fields2cover::types::CoordTransform
 * @brief Reprojects coordinates between WGS84/ETRS89 lon-lat and UTM.
 */
class CoordTransform {
 public:
  CoordTransform(const std::string& from_crs, const std::string& to_crs);
  /**
   * @brief Transform one coordinate in place.
   * @param[in,out] x X or longitude.
   * @param[in,out] y Y or latitude.
   * @param[in,out] z Height, unused.
   * @return True when the transform is defined.
   */
  bool Transform(double* x, double* y, double* z) const;

 private:
  enum class Frame { kIdentity, kLonLat, kUtm };
  struct Spec {
    Frame frame{Frame::kIdentity};
    int zone{0};
    bool north{true};
  };
  Spec from_;
  Spec to_;

  static Spec Parse(const std::string& crs);
  static void LonLatToUtm(double lon, double lat, int zone, bool north,
                          double* easting, double* northing);
  static void UtmToLonLat(double easting, double northing, int zone, bool north,
                          double* lon, double* lat);
};

/**
 * @class autonomy::planning::planner::fields2cover::types::NativeGeom
 * @brief Recursive point / curve / polygon geometry without GDAL.
 */
class NativeGeom {
 public:
  WkbKind kind{WkbKind::kUnknown};
  std::vector<Coord> coords;
  std::vector<std::shared_ptr<NativeGeom>> parts;

  NativeGeom() = default;
  NativeGeom(double x, double y, double z = 0.0);

  NativeGeom* clone() const;
  WkbKind getGeometryType() const { return kind; }

  bool IsEmpty() const;
  bool Equals(const NativeGeom* other) const;

  void getEnvelope(Envelope* envelope) const;

  double getX() const;
  double getY() const;
  double getZ() const;
  double getX(int i) const;
  double getY(int i) const;
  double getZ(int i) const;
  void setX(double x);
  void setY(double y);
  void setZ(double z);

  int getNumPoints() const;
  void addPoint(double x, double y, double z = 0.0);
  void setPoint(int i, double x, double y, double z = 0.0);
  void getPoint(int i, NativeGeom* out) const;
  void reversePoints();
  void removePoint(int i);

  int getNumGeometries() const;
  NativeGeom* getGeometryRef(int i);
  const NativeGeom* getGeometryRef(int i) const;
  void addGeometry(const NativeGeom* geom);

  NativeGeom* getExteriorRing();
  const NativeGeom* getExteriorRing() const;
  NativeGeom* getInteriorRing(int i);
  const NativeGeom* getInteriorRing(int i) const;
  int getNumInteriorRings() const;
  void addRing(const NativeGeom* ring);

  NativeGeom* toPolygon() { return this; }
  const NativeGeom* toPolygon() const { return this; }
  NativeGeom* toMultiPolygon() { return this; }
  const NativeGeom* toMultiPolygon() const { return this; }
  NativeGeom* toMultiLineString() { return this; }
  const NativeGeom* toMultiLineString() const { return this; }

  double get_Length() const;
  double get_Area() const;
  bool isClockwise() const;
  bool get_IsClosed() const;

  double Distance(const NativeGeom* other) const;
  bool Disjoint(const NativeGeom* other) const;
  bool Crosses(const NativeGeom* other) const;
  bool Touches(const NativeGeom* other) const;
  bool Within(const NativeGeom* other) const;
  bool Contains(const NativeGeom* other) const;
  bool Intersects(const NativeGeom* other) const;

  NativeGeom* Intersection(const NativeGeom* other) const;
  NativeGeom* Difference(const NativeGeom* other) const;
  NativeGeom* Union(const NativeGeom* other) const;
  NativeGeom* UnionCascaded() const;
  NativeGeom* ConvexHull() const;
  NativeGeom* Buffer(double dist, int side = 0) const;
  NativeGeom* SimplifyPreserveTopology(double d_tol) const;

  void exportToWkt(char** wkt) const;
  void importFromWkt(const char** wkt);
  char* exportToGML() const;
  char* exportToKML() const;
  char* exportToJson() const;

  void transform(const CoordTransform* ct);

  NativeGeom& operator=(const NativeGeom&) = default;
};

/**
 * @class autonomy::planning::planner::fields2cover::types::GeomFactory
 * @brief Allocates and frees NativeGeom instances.
 */
class GeomFactory {
 public:
  static NativeGeom* createGeometry(WkbKind kind);
  static NativeGeom* createFromWkt(const std::string& wkt);
  static void destroyGeometry(NativeGeom* geom);
};

inline WkbKind wkbFlatten(WkbKind kind) { return kind; }

}  // namespace autonomy::planning::planner::fields2cover::types
