/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_default_plugins InteractiveMarkerDisplay
 * (Willow Garage / Open Source Robotics Foundation).
 *****************************************************************************/

/**
 * @file interactive_marker_display.hpp
 * @brief InteractiveMarkers Display (RViz InteractiveMarkerDisplay).
 *
 * Subscribes to InteractiveMarker update/init channels, owns per-marker
 * @ref InteractiveMarker instances, and syncs poses into
 * @ref InteractiveMarkerRegistry for the Interact tool.
 *
 * @see InteractiveMarker
 * @see InteractiveMarkerRegistry
 */

#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <automsgs/msgs/visualization_msgs/interactive_marker_feedback.pb.h>
#include <automsgs/msgs/visualization_msgs/interactive_marker_init.pb.h>
#include <automsgs/msgs/visualization_msgs/interactive_marker_update.pb.h>
#include "autolink/message/raw_message.hpp"
#include "autolink/node/reader.hpp"
#include "autoviz/plugins/displays/channel_display.hpp"
#include "autoviz/plugins/displays/interactive_markers/interactive_marker.hpp"
#include "autoviz/plugins/displays/interactive_markers/interactive_marker_registry.hpp"
#include "autoviz/common/interaction/message_queue.hpp"

namespace autoviz {
namespace display {

/**
 * @class InteractiveMarkerDisplay
 * @brief Channel display for visualization_msgs InteractiveMarker updates.
 */
class InteractiveMarkerDisplay
    : public ChannelDisplay<automsgs::msgs::visualization_msgs::
                                InteractiveMarkerUpdate> {
 public:
  explicit InteractiveMarkerDisplay(std::string channel);

  std::string typeId() const override { return "InteractiveMarkers"; }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

  void setRegistry(InteractiveMarkerRegistry* registry) {
    registry_ = registry;
  }

 protected:
  void onEnable() override;
  void onDisable() override;
  void onUpdate() override;
  void processMessage(
      const automsgs::msgs::visualization_msgs::InteractiveMarkerUpdate&
          message) override;
  void clearReceivedData() override;
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  void syncRegistry();
  std::string sourceId() const;
  std::string feedbackChannel() const;
  void publishFeedback(
      automsgs::msgs::visualization_msgs::InteractiveMarkerFeedback& feedback);
  void onStatusUpdate(int level, const std::string& name,
                      const std::string& text);

  void updateMarkers(
      const google::protobuf::RepeatedPtrField<
          automsgs::msgs::visualization_msgs::InteractiveMarker>& markers);
  void updatePoses(
      const google::protobuf::RepeatedPtrField<
          automsgs::msgs::visualization_msgs::InteractiveMarkerPose>&
          marker_poses);
  void eraseMarkers(const std::vector<std::string>& names);
  void eraseAllMarkers();
  void applyShowFlags(InteractiveMarker* marker) const;

  /** Pull Interact-tool pose edits from the registry into owned markers. */
  void pullPosesFromRegistry();

  InteractiveMarkerRegistry* registry_ = nullptr;
  std::map<std::string, InteractiveMarker::SharedPtr> interactive_markers_map_;

  std::shared_ptr<autolink::Reader<autolink::message::RawMessage>> init_reader_;
  integration::MessageQueue init_queue_;
};

}  // namespace display
}  // namespace autoviz
