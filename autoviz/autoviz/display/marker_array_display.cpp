/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/marker_array_display.hpp"

namespace autoviz {
namespace display {

MarkerArrayDisplay::MarkerArrayDisplay(std::string channel)
    : ChannelDisplay<
          automsgs::msgs::visualization_msgs::MarkerArray>(
          "MarkerArray", std::move(channel),
          "automsgs.msgs.visualization_msgs.MarkerArray") {
  setProperties({});
}

std::vector<common::DisplayPropertySpec> MarkerArrayDisplay::propertySpecs()
    const {
  using common::DisplayPropertyKind;
  return {{"color", "Color", "200;200;200", {}, DisplayPropertyKind::kColor},
          {"use_custom_color", "Use Custom Color", "false"},
          {"point_size", "Size (Pixels)", "3", {}},
          {"mesh_style", "Mesh Style", "solid", {"solid", "wireframe"}}};
}

void MarkerArrayDisplay::onPropertyChanged(const std::string& /*key*/) {
  if (context_ != nullptr && context_->request_redraw) {
    context_->request_redraw();
  }
}

void MarkerArrayDisplay::processMessage(
    const automsgs::msgs::visualization_msgs::MarkerArray& message) {
  for (const auto& marker : message.markers()) {
    upsertMarker(marker, &markers_);
  }
  if (context_ != nullptr && context_->request_redraw) {
    context_->request_redraw();
  }
}

void MarkerArrayDisplay::onDraw(rendering::SceneOverlay& scene) {
  drawStoredMarkers(scene, context_, properties(), markers_, name());
}

}  // namespace display
}  // namespace autoviz
