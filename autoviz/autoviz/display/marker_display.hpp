/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <map>

#include <automsgs/msgs/visualization_msgs/marker.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/channel_display.hpp"
#include "autoviz/display/marker_draw_utils.hpp"

namespace autoviz {
namespace display {

class MarkerDisplay
    : public ChannelDisplay<
          automsgs::msgs::visualization_msgs::Marker> {
 public:
  explicit MarkerDisplay(std::string channel);

  std::string typeId() const override { return "Marker"; }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;
  void onPropertyChanged(const std::string& key) override;

 protected:
  void processMessage(
      const automsgs::msgs::visualization_msgs::Marker& message)
      override;
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  std::map<MarkerKey, StoredMarker> markers_;
};

}  // namespace display
}  // namespace autoviz
