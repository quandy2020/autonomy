/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QColor>
#include <QVector3D>
#include <vector>

#include <automsgs/msgs/strata_msgs/semantic_zone.pb.h>
#include "autoviz/display/channel_display.hpp"

namespace autoviz {
namespace display {

class StrataSemanticZoneDisplay
    : public ChannelDisplay<automsgs::msgs::strata_msgs::SemanticZoneArray> {
 public:
  explicit StrataSemanticZoneDisplay(std::string channel);

  std::string typeId() const override { return "StrataSemanticZone"; }
  std::vector<common::DisplayPropertySpec> propertySpecs() const override;
  void onPropertyChanged(const std::string& key) override;

 protected:
  void processMessage(
      const automsgs::msgs::strata_msgs::SemanticZoneArray& message) override;
  void clearReceivedData() override;
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  struct StoredZone {
    std::vector<QVector3D> polygon;
    QColor fill_color;
    QColor outline_color;
    float outline_width{0.02f};
    QString label;
    std::string zone_type;
    std::vector<float> dash_pattern;
  };

  void DrawZoneFill(rendering::SceneOverlay& scene, const StoredZone& zone) const;
  void DrawZoneOutline(rendering::SceneOverlay& scene, const StoredZone& zone) const;

  std::vector<StoredZone> zones_;
};

}  // namespace display
}  // namespace autoviz
