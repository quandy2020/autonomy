/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/interactive_marker_display.hpp"

#include "autolink/message/raw_message.hpp"
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/interactive_marker_registry.hpp"

namespace autoviz {
namespace display {

InteractiveMarkerDisplay::InteractiveMarkerDisplay(std::string channel)
    : ChannelDisplay<
          automsgs::msgs::visualization_msgs::InteractiveMarkerUpdate>(
          "InteractiveMarkers", std::move(channel),
          "autonomy.commsgs.proto.visualization_msgs.InteractiveMarkerUpdate") {}

std::vector<common::DisplayPropertySpec> InteractiveMarkerDisplay::propertySpecs()
    const {
  return {{"feedback_channel", "Feedback Topic", "", {},
           common::DisplayPropertyKind::kChannel},
          {"init_channel", "Init Topic", "", {}, common::DisplayPropertyKind::kChannel},
          {"server_id", "Server ID", "", {}}};
}

std::string InteractiveMarkerDisplay::sourceId() const {
  return name() + "@" + channel();
}

std::string InteractiveMarkerDisplay::feedbackChannel() const {
  const std::string configured =
      propertyValue("feedback_channel", "");
  if (!configured.empty()) {
    return configured;
  }
  return defaultFeedbackChannel(channel());
}

void InteractiveMarkerDisplay::onEnable() {
  ChannelDisplay<
      automsgs::msgs::visualization_msgs::InteractiveMarkerUpdate>::
      onEnable();
  const std::string init_channel = propertyValue("init_channel", "");
  if (init_channel.empty() || context_ == nullptr ||
      context_->autolink == nullptr || context_->autolink->node() == nullptr) {
    return;
  }
  init_reader_ =
      context_->autolink->node()->CreateReader<autolink::message::RawMessage>(
          init_channel,
          [this](const std::shared_ptr<autolink::message::RawMessage>& msg) {
            if (msg != nullptr) {
              init_queue_.push(msg->message);
            }
          });
}

void InteractiveMarkerDisplay::onDisable() {
  init_reader_.reset();
  if (registry_ != nullptr) {
    registry_->removeSource(sourceId());
  }
  markers_.clear();
  ChannelDisplay<
      automsgs::msgs::visualization_msgs::InteractiveMarkerUpdate>::
      onDisable();
}

void InteractiveMarkerDisplay::syncRegistry() {
  if (registry_ != nullptr) {
    registry_->setSourceMarkers(sourceId(), markers_);
  }
}

void InteractiveMarkerDisplay::processMessage(
    const automsgs::msgs::visualization_msgs::InteractiveMarkerUpdate&
        message) {
  const std::string server_id = propertyValue("server_id", message.server_id());
  applyInteractiveMarkerUpdate(message, server_id, feedbackChannel(),
                               sourceId(), &markers_);
  syncRegistry();
  if (context_ != nullptr && context_->request_redraw) {
    context_->request_redraw();
  }
}

void InteractiveMarkerDisplay::onUpdate() {
  while (auto payload = init_queue_.pop()) {
    automsgs::msgs::visualization_msgs::InteractiveMarkerInit init;
    if (!init.ParseFromString(*payload)) {
      continue;
    }
    markers_.clear();
    for (const auto& marker : init.markers()) {
      InteractiveMarkerState state;
      state.server_id = propertyValue("server_id", init.server_id());
      state.feedback_channel = feedbackChannel();
      state.source_id = sourceId();
      state.marker = marker;
      markers_[marker.name()] = std::move(state);
    }
    syncRegistry();
  }
  ChannelDisplay<
      automsgs::msgs::visualization_msgs::InteractiveMarkerUpdate>::
      onUpdate();
}

void InteractiveMarkerDisplay::onDraw(rendering::SceneOverlay& scene) {
  if (registry_ != nullptr) {
    for (const auto& [marker_name, state] : registry_->markers()) {
      if (state.source_id != sourceId()) {
        continue;
      }
      drawInteractiveMarker(scene, context_, properties(), state);
    }
    return;
  }
  for (const auto& [marker_name, state] : markers_) {
    (void)marker_name;
    drawInteractiveMarker(scene, context_, properties(), state);
  }
}

}  // namespace display
}  // namespace autoviz
