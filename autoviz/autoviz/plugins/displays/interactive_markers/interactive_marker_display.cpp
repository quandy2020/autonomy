/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_default_plugins InteractiveMarkerDisplay
 * (Willow Garage / Open Source Robotics Foundation).
 *****************************************************************************/

#include "autoviz/plugins/displays/interactive_markers/interactive_marker_display.hpp"

#include <vector>

#include "autolink/message/raw_message.hpp"
#include "autoviz/common/display_property.hpp"
#include "autoviz/plugins/displays/interactive_markers/interactive_marker_namespace_property.hpp"
#include "autoviz/plugins/tools/common/publish_tool_utils.hpp"

namespace autoviz {
namespace display {

InteractiveMarkerDisplay::InteractiveMarkerDisplay(std::string channel)
    : ChannelDisplay<
          automsgs::msgs::visualization_msgs::InteractiveMarkerUpdate>(
          "InteractiveMarkers", std::move(channel),
          "automsgs.msgs.visualization_msgs.InteractiveMarkerUpdate") {}

std::vector<common::DisplayPropertySpec>
InteractiveMarkerDisplay::propertySpecs() const {
  return {{"feedback_channel", "Feedback Topic", "", {},
           common::DisplayPropertyKind::kChannel},
          {"init_channel", "Init Topic", "", {},
           common::DisplayPropertyKind::kChannel},
          {"server_id", "Server ID", "", {}},
          {"show_descriptions", "Show Descriptions", "true",
           {"true", "false"}},
          {"show_axes", "Show Axes", "false", {"true", "false"}},
          {"show_visual_aids", "Show Visual Aids", "false",
           {"true", "false"}}};
}

std::string InteractiveMarkerDisplay::sourceId() const {
  return name() + "@" + channel();
}

std::string InteractiveMarkerDisplay::feedbackChannel() const {
  const std::string configured = propertyValue("feedback_channel", "");
  if (!configured.empty()) {
    return configured;
  }
  const std::string ns =
      InteractiveMarkerNamespaceProperty::namespaceFromUpdateChannelStd(
          channel());
  if (!ns.empty()) {
    return ns + "/feedback";
  }
  return defaultFeedbackChannel(channel());
}

void InteractiveMarkerDisplay::applyShowFlags(InteractiveMarker* marker) const {
  if (marker == nullptr) {
    return;
  }
  marker->setShowDescription(propertyValue("show_descriptions", "true") !=
                             "false");
  marker->setShowAxes(propertyValue("show_axes", "false") == "true");
  marker->setShowVisualAids(propertyValue("show_visual_aids", "false") ==
                            "true");
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
  eraseAllMarkers();
  if (registry_ != nullptr) {
    registry_->removeSource(sourceId());
  }
  ChannelDisplay<
      automsgs::msgs::visualization_msgs::InteractiveMarkerUpdate>::
      onDisable();
}

void InteractiveMarkerDisplay::publishFeedback(
    automsgs::msgs::visualization_msgs::InteractiveMarkerFeedback& feedback) {
  if (context_ == nullptr || context_->autolink == nullptr ||
      context_->autolink->node() == nullptr) {
    return;
  }
  const std::string topic = feedbackChannel();
  if (topic.empty()) {
    return;
  }
  tools::PublishMessage(context_->autolink->node(), topic, feedback);
}

void InteractiveMarkerDisplay::onStatusUpdate(int level, const std::string& name,
                                              const std::string& text) {
  (void)level;
  (void)name;
  (void)text;
}

void InteractiveMarkerDisplay::updateMarkers(
    const google::protobuf::RepeatedPtrField<
        automsgs::msgs::visualization_msgs::InteractiveMarker>& markers) {
  for (const auto& marker : markers) {
    auto entry = interactive_markers_map_.find(marker.name());
    if (entry == interactive_markers_map_.end()) {
      auto created = std::make_shared<InteractiveMarker>(context_);
      // InteractiveMarker is a QObject; Display is not — use marker as context.
      QObject::connect(
          created.get(), &InteractiveMarker::userFeedback, created.get(),
          [this](automsgs::msgs::visualization_msgs::InteractiveMarkerFeedback&
                     feedback) { publishFeedback(feedback); });
      QObject::connect(
          created.get(), &InteractiveMarker::statusUpdate, created.get(),
          [this](int level, const std::string& name, const std::string& text) {
            onStatusUpdate(level, name, text);
          });
      entry =
          interactive_markers_map_.emplace(marker.name(), std::move(created))
              .first;
    }
    if (entry->second->processMessage(marker)) {
      applyShowFlags(entry->second.get());
    }
  }
}

void InteractiveMarkerDisplay::updatePoses(
    const google::protobuf::RepeatedPtrField<
        automsgs::msgs::visualization_msgs::InteractiveMarkerPose>&
        marker_poses) {
  for (const auto& pose_update : marker_poses) {
    auto entry = interactive_markers_map_.find(pose_update.name());
    if (entry != interactive_markers_map_.end()) {
      entry->second->processMessage(pose_update);
    }
  }
}

void InteractiveMarkerDisplay::eraseMarkers(
    const std::vector<std::string>& names) {
  for (const auto& marker_name : names) {
    interactive_markers_map_.erase(marker_name);
  }
}

void InteractiveMarkerDisplay::eraseAllMarkers() {
  interactive_markers_map_.clear();
}

void InteractiveMarkerDisplay::syncRegistry() {
  if (registry_ == nullptr) {
    return;
  }
  std::map<std::string, InteractiveMarkerState> markers;
  const std::string server_id = propertyValue("server_id", "");
  const std::string feedback = feedbackChannel();
  const std::string source = sourceId();
  for (const auto& [name, marker] : interactive_markers_map_) {
    InteractiveMarkerState state;
    state.server_id = server_id;
    state.feedback_channel = feedback;
    state.source_id = source;
    state.marker = marker->toMessage();
    markers[name] = std::move(state);
  }
  registry_->setSourceMarkers(source, markers);
}

void InteractiveMarkerDisplay::pullPosesFromRegistry() {
  if (registry_ == nullptr) {
    return;
  }
  for (auto& [name, marker] : interactive_markers_map_) {
    const auto it = registry_->markers().find(name);
    if (it == registry_->markers().end()) {
      continue;
    }
    const auto& pose = it->second.marker.pose();
    marker->setPose(
        QVector3D(static_cast<float>(pose.position().x()),
                  static_cast<float>(pose.position().y()),
                  static_cast<float>(pose.position().z())),
        QQuaternion(static_cast<float>(pose.orientation().w()),
                    static_cast<float>(pose.orientation().x()),
                    static_cast<float>(pose.orientation().y()),
                    static_cast<float>(pose.orientation().z())),
        "");
  }
}

void InteractiveMarkerDisplay::processMessage(
    const automsgs::msgs::visualization_msgs::InteractiveMarkerUpdate&
        message) {
  // KEEP_ALIVE (0) ignored; UPDATE (1) applies markers/poses/erases.
  if (message.type() != 1) {
    return;
  }
  updateMarkers(message.markers());
  updatePoses(message.poses());
  std::vector<std::string> erases;
  erases.reserve(static_cast<std::size_t>(message.erases_size()));
  for (const auto& erased : message.erases()) {
    erases.push_back(erased);
  }
  eraseMarkers(erases);
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
    eraseAllMarkers();
    updateMarkers(init.markers());
    syncRegistry();
  }
  for (auto& [name, marker] : interactive_markers_map_) {
    (void)name;
    marker->update();
  }
  ChannelDisplay<
      automsgs::msgs::visualization_msgs::InteractiveMarkerUpdate>::
      onUpdate();
}

void InteractiveMarkerDisplay::clearReceivedData() {
  eraseAllMarkers();
  init_queue_.clear();
  syncRegistry();
}

void InteractiveMarkerDisplay::onDraw(rendering::SceneOverlay& scene) {
  pullPosesFromRegistry();
  // Apply Interact-tool hover/active highlight (translate=cyan, rotate=amber).
  for (const auto& [name, marker] : interactive_markers_map_) {
    (void)name;
    marker->clearControlHighlights();
  }
  if (registry_ != nullptr) {
    const auto& hl = registry_->highlight();
    if (hl.level != InteractiveMarkerHighlight::Level::kNone &&
        !hl.marker_name.empty()) {
      auto it = interactive_markers_map_.find(hl.marker_name);
      if (it != interactive_markers_map_.end()) {
        InteractiveMarkerControl::ControlHighlight level =
            InteractiveMarkerControl::kNoHighlight;
        if (hl.level == InteractiveMarkerHighlight::Level::kHover) {
          level = InteractiveMarkerControl::kHoverHighlight;
        } else if (hl.level == InteractiveMarkerHighlight::Level::kActive) {
          level = InteractiveMarkerControl::kActiveHighlight;
        }
        InteractiveMarkerControl::HighlightKind kind =
            InteractiveMarkerControl::HighlightKind::kAuto;
        switch (hl.kind) {
          case InteractiveMarkerHighlight::Kind::kTranslate:
            kind = InteractiveMarkerControl::HighlightKind::kTranslate;
            break;
          case InteractiveMarkerHighlight::Kind::kRotate:
            kind = InteractiveMarkerControl::HighlightKind::kRotate;
            break;
          case InteractiveMarkerHighlight::Kind::kBoth:
            kind = InteractiveMarkerControl::HighlightKind::kBoth;
            break;
          default:
            break;
        }
        it->second->setControlHighlight(hl.control_name, level, kind);
      }
    }
  }
  for (const auto& [name, marker] : interactive_markers_map_) {
    (void)name;
    applyShowFlags(marker.get());
    marker->draw(scene, properties());
  }
}

}  // namespace display
}  // namespace autoviz
