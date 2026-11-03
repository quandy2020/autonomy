/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <string>
#include <vector>

#include <QColor>
#include <QVector3D>

#include <automsgs/msgs/visualization_msgs/pictogram.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/plugins/displays/channel_display.hpp"

namespace autoviz {
namespace display {

class PictogramDisplay
    : public ChannelDisplay<automsgs::msgs::visualization_msgs::Pictogram> {
 public:
  explicit PictogramDisplay(std::string channel);

  std::string typeId() const override { return "Pictogram"; }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  void processMessage(
      const automsgs::msgs::visualization_msgs::Pictogram& message) override;
  void clearReceivedData() override;
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  struct LabelVisual {
    std::string text;
    QVector3D position;
    QColor color;
    float char_height;
  };

  std::vector<LabelVisual> labels_;
};

}  // namespace display
}  // namespace autoviz
