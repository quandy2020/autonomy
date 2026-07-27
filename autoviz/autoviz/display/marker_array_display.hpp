/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <map>

#include <automsgs/msgs/visualization_msgs/marker_array.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/channel_display.hpp"
#include "autoviz/display/marker_draw_utils.hpp"

namespace autoviz {
namespace display {

class MarkerArrayDisplay
    : public ChannelDisplay<
          automsgs::msgs::visualization_msgs::MarkerArray> {
 public:
  explicit MarkerArrayDisplay(std::string channel);

  std::string typeId() const override { return "MarkerArray"; }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;
  void onPropertyChanged(const std::string& key) override;

 protected:
  void processMessage(
      const automsgs::msgs::visualization_msgs::MarkerArray& message)
      override;
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  std::map<MarkerKey, StoredMarker> markers_;
};

}  // namespace display
}  // namespace autoviz
