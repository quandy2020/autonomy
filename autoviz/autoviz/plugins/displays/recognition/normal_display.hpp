/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Draws short normal segments from sensor_msgs/PointCloud2 when fields exist.
 *****************************************************************************/

#pragma once

#include <string>
#include <vector>

#include <QColor>
#include <QVector3D>

#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/plugins/displays/channel_display.hpp"

namespace autoviz {
namespace display {

class NormalDisplay
    : public ChannelDisplay<automsgs::msgs::sensor_msgs::PointCloud2> {
 public:
  explicit NormalDisplay(std::string channel);

  std::string typeId() const override { return "Normal"; }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  void processMessage(
      const automsgs::msgs::sensor_msgs::PointCloud2& message) override;
  void clearReceivedData() override;
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  struct LineVisual {
    QVector3D start;
    QVector3D end;
    QColor color;
  };

  std::vector<LineVisual> segments_;
  bool normals_available_ = false;
};

}  // namespace display
}  // namespace autoviz
