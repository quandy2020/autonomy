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
 * @file parser.cpp
 * @brief Fields2Cover parser implementation.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */



#include <cstring>
#include <fstream>
#include <locale>
#include <stdexcept>
#include <string>
#include <pugixml.hpp>
#include <nlohmann/json.hpp>
#include "autonomy/planning/planner/fields2cover/utils/parser.hpp"
#include "autonomy/planning/planner/fields2cover/utils/transformation.hpp"

using json = nlohmann::json;

namespace autonomy {
namespace planning {
namespace planner {
namespace fields2cover {

namespace {

pugi::xml_node ChildByLocalName(const pugi::xml_node& parent,
                                const char* local_name) {
  for (pugi::xml_node child : parent.children()) {
    const char* name = child.name();
    const char* colon = std::strrchr(name, ':');
    const char* local = colon ? colon + 1 : name;
    if (std::strcmp(local, local_name) == 0) {
      return child;
    }
  }
  return {};
}

}  // namespace

void Parser::ImportGml(const std::string& file, F2CFields& fields) {
  fields.emplace_back(ImportFieldGml(file));
}

F2CField Parser::ImportFieldGml(
    const std::string& file, bool coord_frame_fail_silently) {
  std::locale::global(std::locale::classic());

  pugi::xml_document doc;
  const pugi::xml_parse_result loaded = doc.load_file(file.c_str());
  if (!loaded) {
    throw std::invalid_argument("File not found");
  }
  pugi::xml_node p_parcel = doc.document_element();
  if (!p_parcel) {
    throw std::invalid_argument("File not found");
  }

  std::string id {""};
  if (pugi::xml_attribute id_attr = p_parcel.attribute("id")) {
    id = id_attr.as_string();
  }

  pugi::xml_node p_field = ChildByLocalName(p_parcel, "Field");
  pugi::xml_node p_polygon = ChildByLocalName(
      ChildByLocalName(p_field, "geometry"), "Polygon");

  std::string coord_sys = p_polygon.attribute("SrsName").as_string();

  std::string p_coords = ChildByLocalName(
      ChildByLocalName(
          ChildByLocalName(p_polygon, "OuterBoundaryIs"), "LinearRing"),
      "coordinates").child_value();
  auto FindAndReplaceAll =
      [](std::string& data, std::string ToSearch, std::string ReplaceStr) {
            size_t pos = data.find(ToSearch);
            while (pos != std::string::npos) {
                data.replace(pos, ToSearch.size(), ReplaceStr);
                pos = data.find(ToSearch, pos + ReplaceStr.size());
            }
            return;
      };
  FindAndReplaceAll(p_coords, ",", ";");
  FindAndReplaceAll(p_coords, " ", ", ");
  FindAndReplaceAll(p_coords, ";", " ");
  p_coords = "POLYGON ((" + p_coords + "))";
  OGRGeometry* new_geom{};
  auto spt_ref = Transform::CreateSptRef(coord_sys, coord_frame_fail_silently);
  OGRGeometryFactory::createFromWkt(p_coords.c_str(), spt_ref.get(), &new_geom);

  F2CField field(F2CCells(new_geom), id);
  field.SetCRS(coord_sys);
  OGRGeometryFactory::destroyGeometry(new_geom);

  return field;
}


F2CPoint GetPointFromJson(const json& ps) {
  if (ps.size() == 3) {
    return F2CPoint(ps[0], ps[1], ps[2]);
  } else if (ps.size() == 2) {
    return F2CPoint(ps[0], ps[1]);
  }
  return F2CPoint();
}

F2CCell GetCellFromJson(const json& imported_cell) {
  auto JsonToF2CRing = [] (const json& json_ring) {
    F2CLinearRing ring;
    for (auto&& ps : json_ring) {
      ring.AddPoint(GetPointFromJson(ps));
    }
    return ring;
  };
  F2CCell cell;
  auto json_rings = imported_cell["geometry"]["coordinates"];
  for (int i = 0; i < json_rings.size(); ++i) {
    cell.AddRing(JsonToF2CRing(json_rings[i]));
  }
  return cell;
}


int Parser::ImportJson(const std::string& file, F2CFields& fields) {
  std::ifstream f(file);
  json imported_field = json::parse(f);

  for (auto&& imported_cell : imported_field["features"]) {
    fields.emplace_back(
      F2CField(F2CCells(GetCellFromJson(imported_cell)),
        imported_cell["properties"]["Name"]));
  }
  return 0;
}

F2CCell Parser::ImportCellJson(const std::string& file) {
  std::ifstream f(file);
  json imported_field = json::parse(f);
  return GetCellFromJson(imported_field["features"][0]);
}

F2CSwaths Parser::ImportSwathsJson(const std::string& file) {
  std::ifstream f(file);
  json imported_swaths = json::parse(f);
  F2CSwaths swaths;
  for (auto&& imported_swath : imported_swaths["features"]) {
    F2CLineString line;
    for (auto&& ps : imported_swath["geometry"]["coordinates"]) {
      line.AddPoint(GetPointFromJson(ps));
    }
    swaths.emplace_back(line,
      imported_swath["properties"]["width"],
      imported_swath["properties"]["path_id"]);
  }
  return swaths;
}

F2CStrips Parser::ImportStripsJson(const std::string& file) {
  std::ifstream f(file);
  json imported_strips = json::parse(f);
  F2CStrips strips;
  for (auto&& imported_strip : imported_strips["features"]) {
    F2CStrip strip;
    strip.SetName(imported_strip["properties"]["crop_id"]);
    strip.SetCell(GetCellFromJson(imported_strip));
    strips.emplace_back(strip);
  }
  return strips;
}

}  // namespace fields2cover
}  // namespace planner
}  // namespace planning
}  // namespace autonomy
