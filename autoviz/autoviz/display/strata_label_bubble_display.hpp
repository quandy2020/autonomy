/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QColor>
#include <QVector3D>
#include <vector>

#include <automsgs/msgs/strata_msgs/label_bubble.pb.h>
#include "autoviz/display/channel_display.hpp"

namespace autoviz {
namespace display {

class StrataLabelBubbleDisplay
    : public ChannelDisplay<automsgs::msgs::strata_msgs::LabelBubbleArray> {
 public:
  explicit StrataLabelBubbleDisplay(std::string channel);

  std::string typeId() const override { return "StrataLabelBubble"; }
  std::vector<common::DisplayPropertySpec> propertySpecs() const override;
  void onPropertyChanged(const std::string& key) override;

 protected:
  void processMessage(
      const automsgs::msgs::strata_msgs::LabelBubbleArray& message) override;
  void clearReceivedData() override;
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  struct StoredBubble {
    QVector3D anchor;
    float offset_x{0.f};
    float offset_y{0.f};
    QString text;
  };

  std::vector<StoredBubble> bubbles_;
};

}  // namespace display
}  // namespace autoviz
