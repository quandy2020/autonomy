/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QColor>
#include <QVector3D>
#include <string>
#include <vector>

#include <automsgs/msgs/strata_msgs/iot_bubble.pb.h>
#include "autoviz/display/channel_display.hpp"

namespace autoviz {
namespace display {

class StrataIotBubbleDisplay
    : public ChannelDisplay<automsgs::msgs::strata_msgs::IotBubbleArray> {
 public:
  explicit StrataIotBubbleDisplay(std::string channel);

  std::string typeId() const override { return "StrataIotBubble"; }
  std::vector<common::DisplayPropertySpec> propertySpecs() const override;
  void onPropertyChanged(const std::string& key) override;

 protected:
  void processMessage(
      const automsgs::msgs::strata_msgs::IotBubbleArray& message) override;
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  struct StoredBubble {
    QVector3D position;
    std::string event_type;
    QString message;
  };

  std::vector<StoredBubble> bubbles_;
};

}  // namespace display
}  // namespace autoviz
