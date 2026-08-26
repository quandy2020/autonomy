/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QColor>
#include <QVector3D>
#include <vector>

#include <automsgs/msgs/strata_msgs/poi_marker.pb.h>
#include "autoviz/display/channel_display.hpp"

namespace autoviz {
namespace display {

class StrataPoiDisplay
    : public ChannelDisplay<automsgs::msgs::strata_msgs::PoiMarkerArray> {
 public:
  explicit StrataPoiDisplay(std::string channel);

  std::string typeId() const override { return "StrataPoi"; }
  std::vector<common::DisplayPropertySpec> propertySpecs() const override;
  void onPropertyChanged(const std::string& key) override;

 protected:
  void processMessage(
      const automsgs::msgs::strata_msgs::PoiMarkerArray& message) override;
  void clearReceivedData() override;
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  struct StoredPoi {
    QVector3D position;
    bool selected{false};
    QString label;
  };

  std::vector<StoredPoi> pois_;
};

}  // namespace display
}  // namespace autoviz
