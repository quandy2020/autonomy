/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QColor>
#include <QVector3D>
#include <vector>

#include <automsgs/msgs/strata_msgs/canvas_label.pb.h>
#include "autoviz/display/channel_display.hpp"

namespace autoviz {
namespace display {

class StrataCanvasLabelDisplay
    : public ChannelDisplay<automsgs::msgs::strata_msgs::CanvasLabelArray> {
 public:
  explicit StrataCanvasLabelDisplay(std::string channel);

  std::string typeId() const override { return "StrataCanvasLabel"; }
  std::vector<common::DisplayPropertySpec> propertySpecs() const override;
  void onPropertyChanged(const std::string& key) override;

 protected:
  void processMessage(
      const automsgs::msgs::strata_msgs::CanvasLabelArray& message) override;
  void clearReceivedData() override;
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  struct StoredLabel {
    QVector3D position;
    QColor color;
    QColor halo_color;
    float halo_blur{0.f};
    QString text;
  };

  QColor text_color_{Qt::white};
  QColor halo_color_{QColor(30, 41, 59)};
  float halo_blur_{4.f};
  std::vector<StoredLabel> labels_;
};

}  // namespace display
}  // namespace autoviz
