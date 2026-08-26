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

/** BICMap fill-extrusion 建筑：仅渲染 ns 为 strata_building* 的 Marker。 */
class StrataBuildingDisplay
    : public ChannelDisplay<automsgs::msgs::visualization_msgs::MarkerArray> {
 public:
  explicit StrataBuildingDisplay(std::string channel);

  std::string typeId() const override { return "StrataBuilding"; }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;
  void onPropertyChanged(const std::string& key) override;

 protected:
  void processMessage(
      const automsgs::msgs::visualization_msgs::MarkerArray& message) override;
  void clearReceivedData() override;
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  std::map<MarkerKey, StoredMarker> markers_;
};

}  // namespace display
}  // namespace autoviz
