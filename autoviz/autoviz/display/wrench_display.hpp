/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QVector3D>

#include <automsgs/msgs/geometry_msgs/wrench_stamped.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/channel_display.hpp"

namespace autoviz {
namespace display {

class WrenchDisplay
    : public ChannelDisplay<
          automsgs::msgs::geometry_msgs::WrenchStamped> {
 public:
  explicit WrenchDisplay(std::string channel);

  std::string typeId() const override { return "Wrench"; }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  void processMessage(
      const automsgs::msgs::geometry_msgs::WrenchStamped& message)
      override;
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  bool have_wrench_ = false;
  QVector3D origin_;
  QVector3D force_;
  QVector3D torque_;
};

}  // namespace display
}  // namespace autoviz
