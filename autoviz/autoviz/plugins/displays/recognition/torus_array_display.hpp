/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <string>
#include <vector>

#include <QColor>
#include <QVector3D>

#include <automsgs/msgs/visualization_msgs/torus_array.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/plugins/displays/channel_display.hpp"

namespace autoviz {
namespace display {

class TorusArrayDisplay
    : public ChannelDisplay<automsgs::msgs::visualization_msgs::TorusArray> {
 public:
  explicit TorusArrayDisplay(std::string channel);

  std::string typeId() const override { return "TorusArray"; }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  void processMessage(
      const automsgs::msgs::visualization_msgs::TorusArray& message) override;
  void clearReceivedData() override;
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  struct LineVisual {
    QVector3D start;
    QVector3D end;
    QColor color;
  };

  std::vector<LineVisual> segments_;
};

}  // namespace display
}  // namespace autoviz
