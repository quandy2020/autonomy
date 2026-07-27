/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <set>
#include <string>

#include <automsgs/msgs/tf2_msgs/tf_message.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/channel_display.hpp"

namespace autoviz {
namespace display {

class TfDisplay : public ChannelDisplay<automsgs::msgs::tf2_msgs::TFMessage> {
 public:
  explicit TfDisplay(std::string channel = "/tf");

  std::string typeId() const override { return "TF"; }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  void processMessage(
      const automsgs::msgs::tf2_msgs::TFMessage& message) override;
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  std::set<std::string> frames_;
};

}  // namespace display
}  // namespace autoviz
