// Copyright (c) 2023 Open Navigation LLC; Autonomy adaptation 2026.
// SPDX-License-Identifier: Apache-2.0
#include "autonomy/task/navigation/coverage/coverage_visualizer.hpp"

#include <array>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

namespace autonomy::task::coverage {
namespace {
using Markers = automsgs::msgs::visualization_msgs::MarkerArray;
using Marker = automsgs::msgs::visualization_msgs::Marker;
using Palette = std::array<float, 3>;
using Ring = std::vector<std::pair<double, double>>;

constexpr std::array<Palette, 8> kPalette{{
    {{0.20f, 0.55f, 0.95f}}, {{0.25f, 0.78f, 0.42f}},
    {{0.98f, 0.66f, 0.18f}}, {{0.72f, 0.42f, 0.96f}},
    {{0.12f, 0.82f, 0.82f}}, {{0.96f, 0.42f, 0.62f}},
    {{0.62f, 0.50f, 0.32f}}, {{0.72f, 0.86f, 0.22f}},
}};

/** @brief Create a line strip marker with an explicit colour and height. */
Marker* Line(Markers& markers, const std::string& frame, const std::string& name,
             int id, const Palette& color, float alpha, float width, float z) {
  auto* marker = markers.add_markers();
  marker->mutable_header()->set_frame_id(frame);
  marker->set_ns(name);
  marker->set_id(id);
  marker->set_type(Marker::LINE_STRIP);
  marker->set_action(Marker::ADD);
  marker->mutable_pose()->mutable_orientation()->set_w(1.0);
  marker->mutable_scale()->set_x(width);
  marker->mutable_color()->set_r(color[0]);
  marker->mutable_color()->set_g(color[1]);
  marker->mutable_color()->set_b(color[2]);
  marker->mutable_color()->set_a(alpha);
  marker->mutable_pose()->mutable_position()->set_z(z);
  return marker;
}

/** @brief Ray-casting point-in-ring test used to rasterise a room fill. */
bool Inside(const Ring& ring, double x, double y) {
  bool inside = false;
  for (std::size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++) {
    const auto& a = ring[i];
    const auto& b = ring[j];
    const bool crosses = ((a.second > y) != (b.second > y)) &&
                         (x < (b.first - a.first) * (y - a.second) /
                                      (b.second - a.second) +
                                  a.first);
    if (crosses) {
      inside = !inside;
    }
  }
  return inside;
}

/** @brief Collect closed rings of a cell group without the closing vertex. */
std::vector<Ring> RingsOf(const F2CCells& cells) {
  std::vector<Ring> rings;
  for (const auto& cell : cells) {
    for (const auto& ring : cell) {
      Ring output;
      for (const auto& point : ring) {
        output.emplace_back(point.GetX(), point.GetY());
      }
      if (output.size() > 2 &&
          std::hypot(output.front().first - output.back().first,
                     output.front().second - output.back().second) < 1e-9) {
        output.pop_back();
      }
      if (output.size() >= 3) {
        rings.push_back(std::move(output));
      }
    }
  }
  return rings;
}

/** @brief Even-odd containment across all rings, i.e. outer minus holes. */
bool Contains(const std::vector<Ring>& rings, double x, double y) {
  bool inside = false;
  for (const auto& ring : rings) {
    if (Inside(ring, x, y)) {
      inside = !inside;
    }
  }
  return inside;
}

/**
 * @brief Rasterise a room polygon into one translucent cube-list marker.
 *
 * A single marker per room keeps the marker count low while still tinting the
 * whole walkable area, which makes adjacent rooms readable at a glance.
 */
Marker* Fill(Markers& markers, const std::vector<Ring>& rings,
             const std::string& frame, int id, const Palette& color,
             float alpha, float z) {
  constexpr double kStep = 0.25;
  auto* marker = markers.add_markers();
  marker->mutable_header()->set_frame_id(frame);
  marker->set_ns("region_fill");
  marker->set_id(id);
  marker->set_type(Marker::CUBE_LIST);
  marker->set_action(Marker::ADD);
  marker->mutable_pose()->mutable_orientation()->set_w(1.0);
  marker->mutable_pose()->mutable_position()->set_z(z);
  marker->mutable_scale()->set_x(kStep * 0.96);
  marker->mutable_scale()->set_y(kStep * 0.96);
  marker->mutable_scale()->set_z(0.004);
  marker->mutable_color()->set_r(color[0]);
  marker->mutable_color()->set_g(color[1]);
  marker->mutable_color()->set_b(color[2]);
  marker->mutable_color()->set_a(alpha);
  if (rings.empty()) {
    return marker;
  }
  double min_x = rings.front().front().first;
  double max_x = min_x;
  double min_y = rings.front().front().second;
  double max_y = min_y;
  for (const auto& ring : rings) {
    for (const auto& point : ring) {
      min_x = std::min(min_x, point.first);
      max_x = std::max(max_x, point.first);
      min_y = std::min(min_y, point.second);
      max_y = std::max(max_y, point.second);
    }
  }
  for (double x = min_x + kStep * 0.5; x <= max_x; x += kStep) {
    for (double y = min_y + kStep * 0.5; y <= max_y; y += kStep) {
      if (!Contains(rings, x, y)) {
        continue;
      }
      auto* point = marker->add_points();
      point->set_x(x);
      point->set_y(y);
    }
  }
  return marker;
}
}  // namespace

CoverageVisualizer::CoverageVisualizer(std::shared_ptr<autolink::Node> node,
                                     const std::string& prefix) {
  fields_ = node->CreateWriter<Markers>(prefix + "/coverage/field");
  headlands_ = node->CreateWriter<Markers>(prefix + "/coverage/headland");
  swaths_ = node->CreateWriter<Markers>(prefix + "/coverage/swaths");
  combined_ = node->CreateWriter<Markers>(prefix + "/coverage/markers");
  regions_ = node->CreateWriter<Markers>(prefix + "/coverage/regions");
}

void CoverageVisualizer::Publish(const ComputeAction::Result& result) {
  if (result.error_code() != proto::NONE) {
    return;
  }
  const auto& frame = result.coverage_path().frame_id();
  if (frame.rfind("EPSG:", 0) == 0) {
    return;
  }
  Markers fields;
  Markers headlands;
  Markers swaths;
  Markers regions;
  Markers combined;
  const Palette field_color{{0.20f, 0.62f, 1.00f}};
  const Palette headland_color{{1.00f, 0.68f, 0.18f}};
  const Palette fallback_swath{{0.98f, 0.98f, 1.00f}};
  const Palette global_color{{1.00f, 0.84f, 0.25f}};
  int field_id = 0;
  if (!result.field_wkt().empty()) {
    F2CCells cells;
    cells.ImportFromWkt(result.field_wkt());
    for (const auto& ring : RingsOf(cells)) {
      auto* line = Line(fields, frame, "field", field_id++, field_color, 0.85f,
                        0.05f, 0.03f);
      for (const auto& point : ring) {
        auto* out = line->add_points();
        out->set_x(point.first);
        out->set_y(point.second);
      }
    }
  }
  if (!result.core_wkt().empty()) {
    F2CCells cells;
    cells.ImportFromWkt(result.core_wkt());
    int headland_id = 0;
    for (const auto& ring : RingsOf(cells)) {
      auto* line = Line(headlands, frame, "headland", headland_id++,
                        headland_color, 0.85f, 0.05f, 0.04f);
      for (const auto& point : ring) {
        auto* out = line->add_points();
        out->set_x(point.first);
        out->set_y(point.second);
      }
    }
  }

  std::vector<std::vector<Ring>> room_rings;
  room_rings.reserve(static_cast<std::size_t>(result.regions_size()));
  int outline_id = 0;
  int label_id = 0;
  for (const auto& region : result.regions()) {
    F2CCells cells;
    for (const auto& polygon : region.polygons()) {
      F2CLinearRing ring;
      for (const auto& point : polygon.coordinates()) {
        ring.AddPoint(point.axis1(), point.axis2());
      }
      ring.CloseRing();
      cells.AddGeometry(F2CCell(ring));
    }
    auto rings = RingsOf(cells);
    const auto& color =
        kPalette[static_cast<std::size_t>(std::max(0, region.color_index())) %
                 kPalette.size()];
    Fill(regions, rings, frame, 1000 + label_id, color, 0.28f, 0.012f);
    for (const auto& ring : rings) {
      auto* line = Line(regions, frame, "region", outline_id++, color, 1.0f,
                        0.07f, 0.08f);
      for (const auto& point : ring) {
        auto* out = line->add_points();
        out->set_x(point.first);
        out->set_y(point.second);
      }
    }
    room_rings.push_back(std::move(rings));
    double center_x = 0.0;
    double center_y = 0.0;
    int center_count = 0;
    if (region.polygons_size() > 0) {
      for (const auto& point : region.polygons(0).coordinates()) {
        center_x += point.axis1();
        center_y += point.axis2();
        ++center_count;
      }
    }
    if (center_count > 0) {
      center_x /= static_cast<double>(center_count);
      center_y /= static_cast<double>(center_count);
    }
    auto* label = regions.add_markers();
    label->mutable_header()->set_frame_id(frame);
    label->set_ns("region_label");
    label->set_id(2000 + label_id);
    label->set_type(Marker::TEXT_VIEW_FACING);
    label->set_action(Marker::ADD);
    label->set_text(region.name());
    label->mutable_pose()->mutable_orientation()->set_w(1.0);
    label->mutable_pose()->mutable_position()->set_x(center_x);
    label->mutable_pose()->mutable_position()->set_y(center_y);
    label->mutable_pose()->mutable_position()->set_z(0.55);
    label->mutable_scale()->set_z(0.5);
    label->mutable_color()->set_r(1.0f);
    label->mutable_color()->set_g(1.0f);
    label->mutable_color()->set_b(1.0f);
    label->mutable_color()->set_a(1.0f);
    ++label_id;
  }

  int swath_id = 0;
  for (const auto& swath : result.coverage_path().swaths()) {
    const auto& poses = swath.path().poses();
    if (poses.empty()) {
      continue;
    }
    Palette color = fallback_swath;
    const double x0 = poses.Get(0).pose().position().x();
    const double y0 = poses.Get(0).pose().position().y();
    for (std::size_t index = 0; index < room_rings.size(); ++index) {
      if (Contains(room_rings[index], x0, y0)) {
        const auto& room = kPalette[index % kPalette.size()];
        for (std::size_t channel = 0; channel < 3; ++channel) {
          color[channel] = 0.25f * room[channel] + 0.75f;
        }
        break;
      }
    }
    auto* line = Line(swaths, frame, "swaths", swath_id++, color, 0.95f, 0.09f,
                      0.09f);
    for (const auto& pose : poses) {
      auto* out = line->add_points();
      out->set_x(pose.pose().position().x());
      out->set_y(pose.pose().position().y());
    }
  }

  auto* clear = combined.add_markers();
  clear->set_action(Marker::DELETEALL);
  for (const auto& list : {fields, headlands, regions, swaths}) {
    for (const auto& marker : list.markers()) {
      *combined.add_markers() = marker;
    }
  }
  if (result.nav_path().poses_size() > 1) {
    auto* line = Line(combined, frame, "global_path", 90000, global_color,
                      1.0f, 0.14f, 0.16f);
    for (const auto& pose : result.nav_path().poses()) {
      auto* out = line->add_points();
      out->set_x(pose.pose().position().x());
      out->set_y(pose.pose().position().y());
    }
  }
  fields_->Write(fields);
  headlands_->Write(headlands);
  swaths_->Write(swaths);
  regions_->Write(regions);
  combined_->Write(combined);
}
}  // namespace autonomy::task::coverage
