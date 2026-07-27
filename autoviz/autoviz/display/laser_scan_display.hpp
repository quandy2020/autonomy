/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QColor>
#include <QVector3D>
#include <vector>

#include <automsgs/msgs/sensor_msgs/laser_scan.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/channel_display.hpp"

namespace autoviz {
namespace display {

class LaserScanDisplay
    : public ChannelDisplay<automsgs::msgs::sensor_msgs::LaserScan> {
 public:
  explicit LaserScanDisplay(std::string channel);

  std::string typeId() const override { return "LaserScan"; }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  void processMessage(
      const automsgs::msgs::sensor_msgs::LaserScan& message) override;
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  struct ScanPoint {
    QVector3D position;
    QColor color;
  };
  std::vector<ScanPoint> points_;
};

}  // namespace display
}  // namespace autoviz
