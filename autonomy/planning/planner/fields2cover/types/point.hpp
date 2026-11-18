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
 * @file point.hpp
 * @brief Fields2Cover Point.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */


#include <ogr_geometry.h>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <functional>
#include <memory>
#include <utility>
#include <vector>
#include "autonomy/planning/planner/fields2cover/types/geometry.hpp"

namespace autonomy::planning::planner::fields2cover::types {

/**
 * @class autonomy::planning::planner::fields2cover::types::Point
 * @brief 3D point geometry used throughout Fields2Cover.
 */
struct Point : public Geometry<OGRPoint, wkbPoint> {
 public:
  // Hidden from SWIG, which would also wrap the base copy/move constructors.
#ifndef SWIG
  using Geometry<OGRPoint, wkbPoint>::Geometry;
#endif
  Point();
  Point(double x, double y, double z = 0);
  Point(const Point&);
  Point(Point&&);
  ~Point();
  Point& operator=(const Point&);
  Point& operator=(Point&&);

 public:
  bool operator==(const Point& b) const;

  bool operator!=(const Point& b) const;

  bool operator<(const Point& b) const;

  Point operator+(const Point& b) const;

  Point operator-(const Point& b) const;

  Point& operator*=(double b);

  Point operator*(double b) const;
  double operator*(const Point& b) const;

  Point operator/(double b) const;

  static double det(const Point& u, const Point& v);


 public:
  Point Clone() const;
  double X() const;
  double Y() const;
  double Z() const;

  double GetX() const;
  double GetY() const;
  double GetZ() const;
  void SetX(double x);
  void SetY(double y);
  void SetZ(double z);

  void SetPoint(double x, double y, double z = 0);
  void SetPoint(const OGRPoint& p);
  void SetPoint(const Point& p);

  double GetAngleFromPoints(const Point& end) const;
  double GetAngleFromPoint() const;
  /**
   * @brief Angle between vectors p1->p2 and p2->p3.
   */
  static double GetAngleFromPoints(
      const Point& p1, const Point& p2, const Point& p3);
  Point GetPointFromAngle(double angle, double dist) const;
  /**
   * @brief Point at a Distance along the ray towards another point.
   * Returns this point if both coincide, as no direction is defined then.
   */
  Point GetPointAlong(const Point& to, double dist) const;

  Point RotateFromPoint(double angle, const Point& p_r) const;

  double SignedDistance2Segment(const Point& start, const Point& end) const;
  static Point IntersectionOfLines(
      const Point& l1_s, const Point& l1_e,
      const Point& l2_s, const Point& l2_e);

  Point ClosestPointInSegment(const Point& seg_s, const Point& seg_e) const;

  template <class T>
  std::vector<T> RotateFromPoint(double angle, const std::vector<T>& t) const;

  template <class T>
  T RotateFromPoint(double angle, const T& t) const;
};

std::ostream& operator<<(std::ostream& os, const Point& p);

template <class T>
std::vector<T> Point::RotateFromPoint(
    double angle, const std::vector<T>& t) const {
  std::vector<T> res;
  for (auto&& p : t) {
    res.emplace_back(this->RotateFromPoint(angle, p));
  }
  return res;
}

template <class T>
T Point::RotateFromPoint(double angle, const T& t) const {
  T res;
  for (auto&& p : t) {
    res.AddGeometry(this->RotateFromPoint(angle, p));
  }
  return res;
}


inline OGRPoint operator+(const OGRPoint& a, const autonomy::planning::planner::fields2cover::types::Point& b) {
  return {a.getX() + b.GetX(), a.getY() + b.GetY(), a.getZ() + b.GetZ()};
}

inline OGRPoint operator-(const OGRPoint& a, const autonomy::planning::planner::fields2cover::types::Point& b) {
  return {a.getX() - b.GetX(), a.getY() - b.GetY(), a.getZ() - b.GetZ()};
}

template <class T>
inline std::vector<T> operator+(
    const std::vector<T>& t, const autonomy::planning::planner::fields2cover::types::Point& dir) {
  std::vector<T> res;
  for (auto&& p : t) {
    res.emplace_back(p + dir);
  }
  return res;
}

template <class T>
inline std::vector<T> operator-(
    const autonomy::planning::planner::fields2cover::types::Point& dir, const std::vector<T>& t) {
  std::vector<T> res;
  for (auto&& p : t) {
    res.emplace_back(dir - p);
  }
  return res;
}

template <class T>
inline std::vector<T> operator-(
    const std::vector<T>& t, const autonomy::planning::planner::fields2cover::types::Point& dir) {
  std::vector<T> res;
  for (auto&& p : t) {
    res.emplace_back(p - dir);
  }
  return res;
}

template <class T>
inline T operator+(const T& t, const autonomy::planning::planner::fields2cover::types::Point& dir) {
  T res;
  for (auto&& p : t) {
    res.AddGeometry(p + dir);
  }
  return res;
}

template <class T>
inline T operator-(const autonomy::planning::planner::fields2cover::types::Point& dir, const T& t) {
  T res;
  for (auto&& p : t) {
    res.AddGeometry(dir - p);
  }
  return res;
}

template <class T>
inline T operator-(const T& t, const autonomy::planning::planner::fields2cover::types::Point& dir) {
  T res;
  for (auto&& p : t) {
    res.AddGeometry(p - dir);
  }
  return res;
}

}  // namespace autonomy::planning::planner::fields2cover::types

namespace std {
template<>
struct hash<autonomy::planning::planner::fields2cover::types::Point> {
  inline size_t operator()(const autonomy::planning::planner::fields2cover::types::Point& p) const {
    // Quantized to the tolerance of Point::operator==, so that points which
    // compare equal share a value. Coordinates are Hashed, not summed:
    // the sum overflows to undefined behaviour on negative coordinates.
    const int64_t coords[3] {
      std::llround(p.GetX() * 1e7),
      std::llround(p.GetY() * 1e7),
      std::llround(p.GetZ() * 1e7)};
    size_t h {1469598103934665603ULL};
    for (auto&& c : coords) {
      h = (h ^ static_cast<size_t>(c)) * 1099511628211ULL;
    }
    return h;
  }
};
}

