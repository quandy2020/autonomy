/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QColor>
#include <QVector3D>
#include <vector>

#include <automsgs/msgs/visualization_msgs/marker_array.pb.h>
#include "autoviz/display/channel_display.hpp"

namespace autoviz {
namespace display {

class StrataFovDisplay
    : public ChannelDisplay<automsgs::msgs::visualization_msgs::MarkerArray> {
 public:
  explicit StrataFovDisplay(std::string channel);

  std::string typeId() const override { return "StrataFov"; }
  std::vector<common::DisplayPropertySpec> propertySpecs() const override;
  void onPropertyChanged(const std::string& key) override;

 protected:
  void processMessage(
      const automsgs::msgs::visualization_msgs::MarkerArray& message) override;
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  struct StoredBand {
    std::vector<QVector3D> polygon;
    QColor fill_color;
  };

  std::vector<StoredBand> bands_;
};

}  // namespace display
}  // namespace autoviz
