/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <string>
#include <vector>

#include <QColor>
#include <QVector3D>

#include <automsgs/msgs/visualization_msgs/pictogram_array.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/plugins/displays/channel_display.hpp"

namespace autoviz {
namespace display {

class PictogramArrayDisplay
    : public ChannelDisplay<automsgs::msgs::visualization_msgs::PictogramArray> {
 public:
  explicit PictogramArrayDisplay(std::string channel);

  std::string typeId() const override { return "PictogramArray"; }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  void processMessage(
      const automsgs::msgs::visualization_msgs::PictogramArray& message) override;
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
