/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/marker_display.hpp"

namespace autoviz {
namespace display {

MarkerDisplay::MarkerDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::visualization_msgs::Marker>(
          "Marker", std::move(channel),
          "automsgs.msgs.visualization_msgs.Marker") {
  setProperties({});
}

std::vector<common::DisplayPropertySpec> MarkerDisplay::propertySpecs() const {
  using common::DisplayPropertyKind;
  return {{"color", "Color", "200;200;200", {}, DisplayPropertyKind::kColor},
          {"use_custom_color", "Use Custom Color", "false"},
          {"point_size", "Size (Pixels)", "3", {}},
          {"mesh_style", "Mesh Style", "solid", {"solid", "wireframe"}}};
}

void MarkerDisplay::onPropertyChanged(const std::string& /*key*/) {
  if (context_ != nullptr && context_->request_redraw) {
    context_->request_redraw();
  }
}

void MarkerDisplay::processMessage(
    const automsgs::msgs::visualization_msgs::Marker& message) {
  upsertMarker(message, &markers_);
  if (context_ != nullptr && context_->request_redraw) {
    context_->request_redraw();
  }
}

void MarkerDisplay::onDraw(rendering::SceneOverlay& scene) {
  drawStoredMarkers(scene, context_, properties(), markers_, name());
}

}  // namespace display
}  // namespace autoviz
