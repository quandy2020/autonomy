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
 * @file transformation.cpp
 * @brief Fields2Cover transformation implementation.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */



#include "autonomy/planning/planner/fields2cover/utils/transformation.hpp"
#include <boost/algorithm/string/predicate.hpp>

namespace autonomy {
namespace planning {
namespace planner {
namespace fields2cover {


std::unique_ptr<types::CoordTransform>
    Transform::GenerateCoordTransf(
        const std::string& coord_sys_from, const std::string& coord_sys_to) {
  if (coord_sys_from.empty() || coord_sys_to.empty()) {
    throw std::invalid_argument("Coordinate system empty");
  }
  return std::make_unique<types::CoordTransform>(coord_sys_from, coord_sys_to);
}

void Transform::transform(F2CField& field, const std::string& coord_sys_to) {
  if (!field.IsEmpty()) {
    field.SetField(field.GetField() + field.GetRefPoint());
    field.GetField()->transform(
        GenerateCoordTransf(field.GetCRS(), coord_sys_to).get());
    field.SetRefPoint(field.GetField().GetCellBorder(0).StartPoint());
    field.SetField(field.GetField() - field.GetRefPoint());
  } else {
    field.SetRefPoint( \
        transform(field.GetRefPoint(), field.GetCRS(), coord_sys_to));
  }
  field.SetPrevCRS(field.GetCRS());
  field.SetCRS(coord_sys_to);
}

F2CRoute Transform::TransformRouteWithFieldRef(const F2CRoute& route,
      const F2CField& field, const std::string& coord_sys_to) {
  auto new_route = route.Clone();
  auto coords = GenerateCoordTransf(field.GetCRS(), coord_sys_to);
  for (size_t i = 0; i < new_route.SizeVectorSwaths(); ++i) {
    new_route.SetSwaths(i, TransformSwathsWithFieldRef(
          new_route.GetSwaths(i), field, coord_sys_to));
  }
  for (size_t i = 0; i < new_route.SizeConnections(); ++i) {
    new_route.SetConnection(i, TransformMultiPointWithFieldRef(
          new_route.GetConnection(i), field, coord_sys_to));
  }
  if (route.GetField()) {
    auto domain = *route.GetField() + field.GetRefPoint();
    domain->transform(coords.get());
    new_route.SetField(domain);
  }
  return new_route;
}

F2CPath Transform::TransformPathWithFieldRef(const F2CPath& path,
      const F2CField& field, const std::string& coord_sys_to) {
  if (path.size() < 1) {return path;}

  auto new_path = path;
  F2CPathState s_end = path.back();
  if (s_end.len > 0.0) {
    s_end.point = path.AtEnd();
    s_end.len = 0.0;
    new_path.AddState(s_end);
  }

  auto coords = GenerateCoordTransf(field.GetCRS(), coord_sys_to);
  for (auto&& s : new_path) {
    s.point = s.point + field.GetRefPoint();
    s.point->transform(coords.get());
  }
  for (size_t i = 0; i < new_path.size() - 1; ++i) {
    new_path[i].len = new_path[i].point.Distance(new_path[i+1].point);
    new_path[i].angle = (new_path[i+1].point - new_path[i].point).GetAngleFromPoint();
  }


  return new_path;
}

F2CMultiPoint Transform::TransformMultiPointWithFieldRef(const F2CMultiPoint& mp,
      const F2CField& field, const std::string& coord_sys_to) {
  F2CMultiPoint new_mp;
  auto coords = GenerateCoordTransf(field.GetCRS(), coord_sys_to);
  for (auto&& p : mp) {
    F2CPoint abs_p = p + field.GetRefPoint();
    abs_p->transform(coords.get());
    new_mp.AddGeometry(abs_p);
  }
  return new_mp;
}

F2CSwath Transform::TransformSwathWithFieldRef(const F2CSwath& swath,
      const F2CField& field, const std::string& coord_sys_to) {
  return F2CSwath(transform(
        swath.GetPath(), field.GetRefPoint(), field.GetCRS(), coord_sys_to),
      swath.GetWidth(), swath.GetId());
}

F2CSwaths Transform::TransformSwathsWithFieldRef(const F2CSwaths& swaths,
      const F2CField& field, const std::string& coord_sys_to) {
  F2CSwaths new_swaths;
  for (auto&& swath : swaths) {
    new_swaths.emplace_back(
        TransformSwathWithFieldRef(swath, field, coord_sys_to));
  }
  return new_swaths;
}

F2CStrip Transform::TransformStrip(const F2CStrip& strip,
      const std::string& coord_sys_from, const std::string& coord_sys_to) {
  F2CStrip new_strip;
  new_strip.SetName(strip.GetName());
  new_strip.SetCell(transform(strip.GetCell(), coord_sys_from, coord_sys_to));
  return new_strip;
}

F2CStrips Transform::TransformStrips(const F2CStrips& strips,
      const std::string& coord_sys_from, const std::string& coord_sys_to) {
  F2CStrips new_strips;
  for (auto&& strip : strips) {
    new_strips.emplace_back(
        TransformStrip(strip, coord_sys_from, coord_sys_to));
  }
  return new_strips;
}

F2CSwaths Transform::TransformSwaths(const F2CSwaths& swaths,
      const std::string& coord_sys_from, const std::string& coord_sys_to) {
  F2CSwaths new_swaths;
  for (auto&& swath : swaths) {
    new_swaths.emplace_back(
        TransformSwath(swath, coord_sys_from, coord_sys_to));
  }
  return new_swaths;
}

F2CSwath Transform::TransformSwath(const F2CSwath& swath,
      const std::string& coord_sys_from, const std::string& coord_sys_to) {
  return F2CSwath(transform(swath.GetPath(), coord_sys_from, coord_sys_to),
                   swath.GetWidth(), swath.GetId());
}

F2CPath Transform::TransformPath(const F2CPath& path,
      const std::string& coord_sys_from, const std::string& coord_sys_to) {
  auto new_path = path;
  auto coords = GenerateCoordTransf(coord_sys_from, coord_sys_to);
  for (auto&& s : new_path) {
    s.point->transform(coords.get());
  }
  return new_path;
}

void Transform::TransformToUTM(F2CField& field, bool IsEtrs89Opt) {
  std::string field_crs = field.GetCRS();
  if (!field_crs.empty() && \
      field_crs != "EPSG:4258" && field_crs != "EPSG:4326") {
    transform(field, "EPSG:4326");
    if (IsEtrs89Opt && \
        (field.GetRefPoint().GetX() <  32.88 && \
         field.GetRefPoint().GetX() > -16.1 && \
         field.GetRefPoint().GetY() <  84.73 && \
         field.GetRefPoint().GetY() >  40.18) ) {
      transform(field, "EPSG:4258");
    }
  } else if (field_crs.empty()) {
    if (field.GetRefPoint() == F2CPoint(0, 0)) {
      field = field.Clone();  // Move field first point to (0, 0)
    }
    if (field.GetRefPoint() == F2CPoint(0, 0)) {
      throw std::out_of_range(
        "Error on TransformToUTM: Coordinates are in a local system");
    }
    if (IsEtrs89Opt && \
        (field.GetRefPoint().GetX() <  32.88 && \
         field.GetRefPoint().GetX() > -16.1 && \
         field.GetRefPoint().GetY() <  84.73 && \
         field.GetRefPoint().GetY() >  40.18) ) {
      field.SetCRS("EPSG:4258");
    } else if (field.GetRefPoint().GetX() <  180. && \
               field.GetRefPoint().GetX() > -180. && \
               field.GetRefPoint().GetY() <   90. && \
               field.GetRefPoint().GetY() >  -90.) {
      field.SetCRS("EPSG:4326");
    } else {
      throw std::invalid_argument(
        "Error on TransformToUTM: Coordinates local system was not recognized");
    }
    field_crs = field.GetCRS();
  }

  std::string utm_zone = std::string("UTM:") + \
      std::to_string(static_cast<size_t>(
            floor((field.GetRefPoint().GetX() + 180) / 6) + 1)) + \
      (field.GetRefPoint().GetY() > 0 ? "N" : "S") + \
      " datum:" + (field.GetCRS() == "EPSG:4258" ? "ETRS89" : "WGS84");
  transform(field, utm_zone);
  field.SetPrevCRS(field_crs);
}


void Transform::TransformToPrevCRS(F2CField& field) {
  if (field.GetPrevCRS().empty()) {
    throw std::invalid_argument(
        "Error in Transform::TransformToUTM. No previous CRS recorded.");
  }
  transform(field, field.GetPrevCRS());
}

F2CRoute Transform::TransformToPrevCRS(
    const F2CRoute& route, const F2CField& field) {
  return TransformRouteWithFieldRef(route, field, field.GetPrevCRS());
}

F2CPath Transform::TransformToPrevCRS(
    const F2CPath& path, const F2CField& field) {
  return TransformPathWithFieldRef(path, field, field.GetPrevCRS());
}

F2CStrip Transform::TransformToPrevCRS(
    const F2CStrip& s, const F2CField& field) {
  return TransformStrip(s, field.GetCRS(), field.GetPrevCRS());
}

F2CStrips Transform::TransformToPrevCRS(
    const F2CStrips& s, const F2CField& field) {
  return TransformStrips(s, field.GetCRS(), field.GetPrevCRS());
}

F2CSwath Transform::TransformToPrevCRS(
    const F2CSwath& s, const F2CField& field) {
  return TransformSwathWithFieldRef(s, field, field.GetPrevCRS());
}

F2CSwaths Transform::TransformToPrevCRS(
    const F2CSwaths& s, const F2CField& field) {
  return TransformSwathsWithFieldRef(s, field, field.GetPrevCRS());
}

F2CPoint Transform::GetRefPointInGPS(const F2CField& field) {
  auto point = field.GetRefPoint();
  point->transform(GenerateCoordTransf(field.GetCRS(), "EPSG:4326").get());
  return point;
}

}  // namespace fields2cover
}  // namespace planner
}  // namespace planning
}  // namespace autonomy
