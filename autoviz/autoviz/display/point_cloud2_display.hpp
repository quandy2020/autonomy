/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QColor>
#include <QVector3D>
#include <vector>

#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/channel_display.hpp"

namespace autoviz {
namespace display {

class PointCloud2Display
    : public ChannelDisplay<automsgs::msgs::sensor_msgs::PointCloud2> {
 public:
  explicit PointCloud2Display(std::string channel);

  std::string typeId() const override { return "PointCloud2"; }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  void processMessage(
      const automsgs::msgs::sensor_msgs::PointCloud2& message)
      override;
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  struct CloudPoint {
    QVector3D position;
    QColor color;
  };
  std::vector<CloudPoint> points_;
};

}  // namespace display
}  // namespace autoviz
