/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/strata_building_display.hpp"

namespace autoviz {
namespace display {
namespace {

bool IsBuildingMarker(const automsgs::msgs::visualization_msgs::Marker& marker) {
  const auto& ns = marker.ns();
  return ns == "strata_building" || ns == "strata_building_extrusion";
}

}  // namespace

StrataBuildingDisplay::StrataBuildingDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::visualization_msgs::MarkerArray>(
          "StrataBuilding", std::move(channel),
          "automsgs.msgs.visualization_msgs.MarkerArray") {
  setProperties({});
}

std::vector<common::DisplayPropertySpec> StrataBuildingDisplay::propertySpecs() const {
  using common::DisplayPropertyKind;
  return {{"color", "Color", "200;200;200", {}, DisplayPropertyKind::kColor},
          {"use_custom_color", "Use Custom Color", "false"},
          {"mesh_style", "Mesh Style", "solid", {"solid", "wireframe"}}};
}

void StrataBuildingDisplay::onPropertyChanged(const std::string& /*key*/) {
  if (context_ != nullptr && context_->request_redraw) {
    context_->request_redraw();
  }
}

void StrataBuildingDisplay::processMessage(
    const automsgs::msgs::visualization_msgs::MarkerArray& message) {
  for (const auto& marker : message.markers()) {
    if (!IsBuildingMarker(marker)) {
      continue;
    }
    upsertMarker(marker, &markers_);
  }
  if (context_ != nullptr && context_->request_redraw) {
    context_->request_redraw();
  }
}

void StrataBuildingDisplay::onDraw(rendering::SceneOverlay& scene) {
  drawStoredMarkers(scene, context_, properties(), markers_, name());
}

}  // namespace display
}  // namespace autoviz
