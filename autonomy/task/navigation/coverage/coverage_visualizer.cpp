// Copyright (c) 2023 Open Navigation LLC; Autonomy adaptation 2026.
// SPDX-License-Identifier: Apache-2.0
#include "autonomy/task/navigation/coverage/coverage_visualizer.hpp"

#include <array>
namespace autonomy::task::coverage {
namespace {
using Markers = automsgs::msgs::visualization_msgs::MarkerArray;
using Marker = automsgs::msgs::visualization_msgs::Marker;
Marker* Line(Markers& markers, const std::string& frame, const std::string& name,
             float red, float green, float blue) {
  auto* marker = markers.add_markers();
  marker->mutable_header()->set_frame_id(frame);
  marker->set_ns(name); marker->set_id(markers.markers_size());
  marker->set_type(Marker::LINE_STRIP); marker->set_action(Marker::ADD);
  marker->mutable_pose()->mutable_orientation()->set_w(1);
  marker->mutable_scale()->set_x(0.035);
  marker->mutable_color()->set_r(red); marker->mutable_color()->set_g(green);
  marker->mutable_color()->set_b(blue); marker->mutable_color()->set_a(1);
  return marker;
}
void Rings(Markers& markers, const std::string& wkt, const std::string& frame,
           const std::string& name, float red, float green, float blue) {
  F2CCells cells; cells.ImportFromWkt(wkt);
  for (const auto& cell : cells) for (const auto& ring : cell) {
    auto* marker = Line(markers, frame, name, red, green, blue);
    for (const auto& point : ring) {
      auto* p = marker->add_points(); p->set_x(point.GetX()); p->set_y(point.GetY());
      p->set_z(0.03);
    }
  }
}
}
CoverageVisualizer::CoverageVisualizer(std::shared_ptr<autolink::Node> node,
                                     const std::string& prefix) {
  fields_ = node->CreateWriter<Markers>(prefix + "/coverage/field");
  headlands_ = node->CreateWriter<Markers>(prefix + "/coverage/headland");
  swaths_ = node->CreateWriter<Markers>(prefix + "/coverage/swaths");
  combined_ = node->CreateWriter<Markers>(prefix + "/coverage/markers");
  regions_ = node->CreateWriter<Markers>(prefix + "/coverage/regions");
}
void CoverageVisualizer::Publish(const ComputeAction::Result& result) {
  if (result.error_code() != proto::NONE) return;
  const auto& frame = result.coverage_path().frame_id();
  if (frame.rfind("EPSG:", 0) == 0) return;
  Markers fields, headlands, swaths, combined;
  Rings(fields, result.field_wkt(), frame, "field", .15f, .55f, 1.f);
  Rings(headlands, result.core_wkt(), frame, "headland", 1.f, .65f, .15f);
  for (const auto& swath : result.coverage_path().swaths()) {
    auto* line = Line(swaths, frame, "swaths", .35f, .8f, .95f);
    for (const auto& pose : swath.path().poses())
      *line->add_points() = pose.pose().position();
  }
  Markers regions;
  {
    static const std::array<std::array<float, 3>, 8> kPalette{{
        {{0.20f, 0.55f, 0.95f}}, {{0.25f, 0.75f, 0.45f}},
        {{0.95f, 0.65f, 0.20f}}, {{0.75f, 0.45f, 0.95f}},
        {{0.15f, 0.80f, 0.80f}}, {{0.95f, 0.45f, 0.65f}},
        {{0.60f, 0.50f, 0.35f}}, {{0.70f, 0.85f, 0.25f}},
    }};
    int region_index = 0;
    for (const auto& region : result.regions()) {
      const auto& color = kPalette[static_cast<std::size_t>(region_index) %
                                   kPalette.size()];
      F2CCells cells;
      for (const auto& polygon : region.polygons()) {
        F2CLinearRing ring;
        for (const auto& point : polygon.coordinates()) {
          ring.AddPoint(point.axis1(), point.axis2());
        }
        ring.CloseRing();
        cells.AddGeometry(F2CCell(ring));
      }
      for (const auto& cell : cells) {
        for (const auto& ring : cell) {
          auto* line = Line(regions, frame, "region", color[0], color[1],
                            color[2]);
          line->mutable_scale()->set_x(0.06);
          for (const auto& point : ring) {
            auto* out = line->add_points();
            out->set_x(point.GetX());
            out->set_y(point.GetY());
            out->set_z(0.06);
          }
        }
      }
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
      label->set_id(1000 + region_index);
      label->set_type(Marker::TEXT_VIEW_FACING);
      label->set_action(Marker::ADD);
      label->set_text(region.name());
      label->mutable_pose()->mutable_orientation()->set_w(1.0);
      label->mutable_pose()->mutable_position()->set_x(center_x);
      label->mutable_pose()->mutable_position()->set_y(center_y);
      label->mutable_pose()->mutable_position()->set_z(0.4);
      label->mutable_scale()->set_z(0.45);
      label->mutable_color()->set_r(1.0f);
      label->mutable_color()->set_g(1.0f);
      label->mutable_color()->set_b(1.0f);
      label->mutable_color()->set_a(1.0f);
      ++region_index;
    }
  }
  auto* clear = combined.add_markers(); clear->set_action(Marker::DELETEALL);

  for (const auto& list : {fields, headlands, regions, swaths})
    for (const auto& marker : list.markers()) *combined.add_markers() = marker;
  fields_->Write(fields); headlands_->Write(headlands); swaths_->Write(swaths);
  regions_->Write(regions);
  combined_->Write(combined);
}
}
