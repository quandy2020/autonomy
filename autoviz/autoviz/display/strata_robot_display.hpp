/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QColor>
#include <QVector3D>
#include <vector>

#include <automsgs/msgs/strata_msgs/robot_marker.pb.h>
#include "autoviz/display/channel_display.hpp"

namespace autoviz {
namespace display {

class StrataRobotDisplay
    : public ChannelDisplay<automsgs::msgs::strata_msgs::RobotMarkerArray> {
 public:
  explicit StrataRobotDisplay(std::string channel);

  std::string typeId() const override { return "StrataRobot"; }
  std::vector<common::DisplayPropertySpec> propertySpecs() const override;
  void onPropertyChanged(const std::string& key) override;

 protected:
  void processMessage(
      const automsgs::msgs::strata_msgs::RobotMarkerArray& message) override;
  void clearReceivedData() override;
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  struct StoredRobot {
    QVector3D position;
    float yaw{0.f};
    std::string status;
    QString label;
    float battery{-1.f};
  };

  std::vector<StoredRobot> robots_;
};

}  // namespace display
}  // namespace autoviz
