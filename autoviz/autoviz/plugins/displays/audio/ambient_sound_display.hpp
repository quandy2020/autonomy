/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <string>
#include <vector>

#include <QColor>
#include <QVector3D>

#include <automsgs/msgs/visualization_msgs/hark_power.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/plugins/displays/channel_display.hpp"

namespace autoviz {
namespace display {

class AmbientSoundDisplay
    : public ChannelDisplay<automsgs::msgs::visualization_msgs::HarkPower> {
 public:
  explicit AmbientSoundDisplay(std::string channel);

  std::string typeId() const override { return "AmbientSound"; }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  void processMessage(
      const automsgs::msgs::visualization_msgs::HarkPower& message) override;
  void clearReceivedData() override;
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  struct ArrowVisual {
    QVector3D start;
    QVector3D end;
    QColor color;
  };

  QVector3D origin_;
  bool have_origin_ = false;
  std::vector<ArrowVisual> arrows_;
};

}  // namespace display
}  // namespace autoviz
