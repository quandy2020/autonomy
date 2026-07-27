/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <deque>

#include <QVector3D>

#include <automsgs/msgs/sensor_msgs/range.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/channel_display.hpp"

namespace autoviz {
namespace display {

class RangeDisplay
    : public ChannelDisplay<automsgs::msgs::sensor_msgs::Range> {
 public:
  explicit RangeDisplay(std::string channel);

  std::string typeId() const override { return "Range"; }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  void processMessage(const automsgs::msgs::sensor_msgs::Range& message)
      override;
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  struct Sample {
    QVector3D origin;
    float range = 0.f;
    float field_of_view = 0.f;
  };
  std::deque<Sample> history_;
};

}  // namespace display
}  // namespace autoviz
