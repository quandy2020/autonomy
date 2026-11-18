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
 * @file transformation.hpp
 * @brief Fields2Cover transformation.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */


#include <ogr_spatialref.h>
#include <memory>
#include <utility>
#include <string>
#include <algorithm>
#include "autonomy/planning/planner/fields2cover/types.hpp"

namespace autonomy {
namespace planning {
namespace planner {
namespace fields2cover {

/**
 * @class autonomy::planning::planner::fields2cover::Transform
 * @brief Reproject Fields2Cover geometries between coordinate systems.
 */
class Transform {
 public:
  static std::unique_ptr<OGRCoordinateTransformation,
    void(*)(OGRCoordinateTransformation*)>
      GenerateCoordTransf(
          const std::string& coord_sys_from, const std::string& coord_sys_to);

  static F2CRoute TransformRouteWithFieldRef(const F2CRoute& route,
      const F2CField& field, const std::string& coord_sys_to);
  static F2CPath TransformPathWithFieldRef(const F2CPath& path,
      const F2CField& field, const std::string& coord_sys_to);
  static F2CSwath TransformSwathWithFieldRef(const F2CSwath& swath,
      const F2CField& field, const std::string& coord_sys_to);
  static F2CSwaths TransformSwathsWithFieldRef(const F2CSwaths& swaths,
      const F2CField& field, const std::string& coord_sys_to);
  static F2CMultiPoint TransformMultiPointWithFieldRef(const F2CMultiPoint& mp,
      const F2CField& field, const std::string& coord_sys_to);


  static void transform(F2CField& field, const std::string& coord_sys_to);

  static void TransformToUTM(F2CField& field, bool IsEtrs89Opt = true);
  static void TransformToPrevCRS(F2CField& field);

  static F2CRoute TransformToPrevCRS(const F2CRoute& p, const F2CField& field);
  static F2CPath TransformToPrevCRS(const F2CPath& p, const F2CField& field);
  static F2CStrip TransformToPrevCRS(const F2CStrip& s, const F2CField& field);
  static F2CStrips TransformToPrevCRS(
      const F2CStrips& s, const F2CField& field);
  static F2CSwath TransformToPrevCRS(
      const F2CSwath& s, const F2CField& field);
  static F2CSwaths TransformToPrevCRS(
      const F2CSwaths& s, const F2CField& field);

  static F2CPath TransformPath(const F2CPath& p,
      const std::string& coord_sys_from, const std::string& coord_sys_to);
  static F2CStrip TransformStrip(const F2CStrip& s,
      const std::string& coord_sys_from, const std::string& coord_sys_to);
  static F2CStrips TransformStrips(const F2CStrips& s,
      const std::string& coord_sys_from, const std::string& coord_sys_to);
  static F2CSwath TransformSwath(const F2CSwath& s,
      const std::string& coord_sys_from, const std::string& coord_sys_to);
  static F2CSwaths TransformSwaths(const F2CSwaths& s,
      const std::string& coord_sys_from, const std::string& coord_sys_to);

  template <class T>
  static T transform(const T& t,
      const std::string& coord_sys_from, const std::string& coord_sys_to);

  template <class T>
  static T transform(const T& t, const F2CPoint& ref_point,
      const std::string& coord_sys_from, const std::string& coord_sys_to);

  static F2CPoint GetRefPointInGPS(const F2CField& field);

  static std::unique_ptr<OGRSpatialReference, void(*)(OGRSpatialReference*)>
        CreateSptRef(const std::string& coord_sys, bool fail_silently = false);

  static std::unique_ptr<OGRCoordinateTransformation,
    void(*)(OGRCoordinateTransformation*)> CreateCoordTransf(
    std::unique_ptr<OGRSpatialReference, void(*)(OGRSpatialReference*)> in,
    std::unique_ptr<OGRSpatialReference, void(*)(OGRSpatialReference*)> out);
};

template <class T>
T Transform::transform(const T& t, const F2CPoint& ref_point,
    const std::string& coord_sys_from, const std::string& coord_sys_to) {
  return transform(t + ref_point, coord_sys_from, coord_sys_to);
}

template <class T>
T Transform::transform(const T& t,
    const std::string& coord_sys_from, const std::string& coord_sys_to) {
  T new_t = t.Clone();
  new_t->transform(GenerateCoordTransf(coord_sys_from, coord_sys_to).get());
  return new_t;
}

}  // namespace fields2cover
}  // namespace planner
}  // namespace planning
}  // namespace autonomy

