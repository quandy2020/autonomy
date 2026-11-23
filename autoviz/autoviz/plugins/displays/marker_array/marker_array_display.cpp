/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/plugins/displays/marker_array/marker_array_display.hpp"

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

void MarkerArrayDisplay::onEnable() {
  if (context_ == nullptr || context_->autolink == nullptr ||
      context_->autolink->node() == nullptr) {
    setStatusError("Autolink not ready");
    return;
  }
  if (channel().empty()) {
    setStatusError("No topic set");
    return;
  }
  pending_state_ = std::make_shared<PendingState>();
  auto state = pending_state_;
  auto redraw = context_->request_redraw;
  typed_reader_ =
      context_->autolink->node()->CreateReader<MarkerArray>(
          channel(), [state, redraw](const std::shared_ptr<MarkerArray>& message) {
            if (!message) return;
            {
              // Keep the shared message instead of deep-copying the whole
              // array on the transport thread, and bail out once the display is
              // gone so a late delivery cannot call into freed memory.
              std::lock_guard<std::mutex> lock(state->mutex);
              if (!state->alive) return;
              state->message = message;
            }
            if (redraw) redraw();
          });
  if (!typed_reader_) {
    setStatusError("Failed to subscribe MarkerArray");
  } else {
    setStatusWarn("Waiting for MarkerArray");
  }
}

void MarkerArrayDisplay::onDisable() {
  typed_reader_.reset();
  if (pending_state_) {
    std::lock_guard<std::mutex> lock(pending_state_->mutex);
    pending_state_->alive = false;
    pending_state_->message.reset();
  }
  ChannelDisplay<MarkerArray>::onDisable();
}

void MarkerArrayDisplay::onUpdate() {
  std::shared_ptr<const MarkerArray> message;
  if (pending_state_) {
    std::lock_guard<std::mutex> lock(pending_state_->mutex);
    message.swap(pending_state_->message);
  }
  if (message) {
    processMessage(*message);
    setStatusOk();
  } else if (typed_reader_ && status().message.empty()) {
    setStatusWarn("Waiting for MarkerArray");
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

void MarkerArrayDisplay::clearReceivedData() { markers_.clear(); }

void MarkerArrayDisplay::onDraw(rendering::SceneOverlay& scene) {
  drawStoredMarkers(scene, context_, properties(), markers_, name());
}

}  // namespace display
}  // namespace autoviz
