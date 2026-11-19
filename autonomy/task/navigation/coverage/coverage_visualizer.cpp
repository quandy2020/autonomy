// Copyright (c) 2023 Open Navigation LLC; Autonomy adaptation 2026.
// SPDX-License-Identifier: Apache-2.0
#include "autonomy/task/navigation/coverage/coverage_visualizer.hpp"
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
  auto* clear = combined.add_markers(); clear->set_action(Marker::DELETEALL);
  for (const auto& list : {fields, headlands, swaths})
    for (const auto& marker : list.markers()) *combined.add_markers() = marker;
  fields_->Write(fields); headlands_->Write(headlands); swaths_->Write(swaths);
  combined_->Write(combined);
}
}
