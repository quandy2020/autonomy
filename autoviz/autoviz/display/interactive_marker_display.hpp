/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <map>
#include <memory>
#include <string>

#include <automsgs/msgs/visualization_msgs/interactive_marker_init.pb.h>
#include <automsgs/msgs/visualization_msgs/interactive_marker_update.pb.h>
#include "autolink/message/raw_message.hpp"
#include "autolink/node/reader.hpp"
#include "autoviz/display/channel_display.hpp"
#include "autoviz/display/interactive_marker_registry.hpp"
#include "autoviz/integration/message_queue.hpp"

namespace autoviz {
namespace display {

class InteractiveMarkerDisplay
    : public ChannelDisplay<automsgs::msgs::visualization_msgs::
                                InteractiveMarkerUpdate> {
 public:
  explicit InteractiveMarkerDisplay(std::string channel);

  std::string typeId() const override { return "InteractiveMarkers"; }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

  void setRegistry(InteractiveMarkerRegistry* registry) { registry_ = registry; }

 protected:
  void onEnable() override;
  void onDisable() override;
  void onUpdate() override;
  void processMessage(
      const automsgs::msgs::visualization_msgs::InteractiveMarkerUpdate&
          message) override;
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  void syncRegistry();
  std::string sourceId() const;
  std::string feedbackChannel() const;

  InteractiveMarkerRegistry* registry_ = nullptr;
  std::map<std::string, InteractiveMarkerState> markers_;
  std::shared_ptr<autolink::Reader<autolink::message::RawMessage>> init_reader_;
  integration::MessageQueue init_queue_;
};

}  // namespace display
}  // namespace autoviz
