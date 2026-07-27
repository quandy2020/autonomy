/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QVector3D>

#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/channel_display.hpp"

namespace autoviz {
namespace display {

class TwistStampedDisplay
    : public ChannelDisplay<automsgs::msgs::geometry_msgs::TwistStamped> {
 public:
  explicit TwistStampedDisplay(std::string channel);

  std::string typeId() const override { return "TwistStamped"; }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  void processMessage(
      const automsgs::msgs::geometry_msgs::TwistStamped& message) override;
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  bool have_twist_ = false;
  QVector3D origin_;
  QVector3D linear_;
  QVector3D angular_;
};

}  // namespace display
}  // namespace autoviz
