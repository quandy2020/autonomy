/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QColor>
#include <QVector3D>
#include <string>
#include <vector>

#include <automsgs/msgs/strata_msgs/robot_3d_layer.pb.h>
#include "autoviz/display/channel_display.hpp"

namespace autoviz {
namespace display {

class StrataRobot3DDisplay
    : public ChannelDisplay<automsgs::msgs::strata_msgs::Robot3DLayerArray> {
 public:
  explicit StrataRobot3DDisplay(std::string channel);

  std::string typeId() const override { return "StrataRobot3D"; }
  std::vector<common::DisplayPropertySpec> propertySpecs() const override;
  void onPropertyChanged(const std::string& key) override;

 protected:
  void processMessage(
      const automsgs::msgs::strata_msgs::Robot3DLayerArray& message) override;
  void clearReceivedData() override;
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  struct StoredLayer {
    QVector3D position;
    float yaw{0.f};
    std::string status;
    QString label;
    QString model_url;
    float scale{1.f};
  };

  std::vector<StoredLayer> layers_;
};

}  // namespace display
}  // namespace autoviz
