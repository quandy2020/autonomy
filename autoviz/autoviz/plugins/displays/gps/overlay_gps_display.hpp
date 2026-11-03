/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Ported from gps_rviz_plugin OverlayGpsDisplay (BSD).
 *****************************************************************************/

/**
 * @file overlay_gps_display.hpp
 * @brief HUD overlay that shows a Google Static Map around NavSatFix fixes.
 */

#pragma once

#include <cstddef>
#include <deque>
#include <memory>
#include <string>

#include <QImage>

#include <automsgs/msgs/sensor_msgs/nav_sat_fix.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/plugins/displays/channel_display.hpp"
#include "autoviz/plugins/displays/common/overlay_object.hpp"

namespace autoviz {
namespace display {

/**
 * @class OverlayGpsDisplay
 * @brief Viewport HUD for @c sensor_msgs/NavSatFix using Google Static Maps.
 *
 * Downloads a map image centered on the latest fix (with a short trajectory
 * path) and paints it via @ref viz::OverlayObject. Requires a non-empty
 * @c api_key property.
 */
class OverlayGpsDisplay
    : public ChannelDisplay<automsgs::msgs::sensor_msgs::NavSatFix> {
 public:
  explicit OverlayGpsDisplay(std::string channel);

  std::string typeId() const override { return "OverlayGps"; }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  void processMessage(
      const automsgs::msgs::sensor_msgs::NavSatFix& message) override;
  void clearReceivedData() override;
  void onEnable() override;
  void onDisable() override;
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  struct FixSample {
    double latitude = 0.0;
    double longitude = 0.0;
  };

  void ensureOverlay();
  void refreshTexture();
  void resizeHistory(std::size_t length);
  bool buildRequestUrl(const FixSample& center, std::string* request_url);
  bool downloadMap(const std::string& request_url, QImage* out_image);

  viz::OverlayObject::Ptr overlay_;
  QImage map_image_;
  bool require_update_texture_ = false;
  std::deque<FixSample> fix_history_;
  std::size_t message_count_ = 0;
};

}  // namespace display
}  // namespace autoviz
