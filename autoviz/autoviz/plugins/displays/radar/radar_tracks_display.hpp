/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file radar_tracks_display.hpp
 * @brief Displays @c sensor_msgs/RadarTracks as boxes + velocity arrows + IDs.
 */

#pragma once

#include <QColor>
#include <QVector3D>
#include <string>
#include <vector>

#include <automsgs/msgs/sensor_msgs/radar.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/plugins/displays/channel_display.hpp"

namespace autoviz {
namespace display {

/**
 * @class RadarTracksDisplay
 * @brief Channel display for RadarTracks object lists (TrafficRadar-style).
 */
class RadarTracksDisplay
    : public ChannelDisplay<automsgs::msgs::sensor_msgs::RadarTracks> {
 public:
  explicit RadarTracksDisplay(std::string channel);

  std::string typeId() const override { return "RadarTracks"; }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  void onPropertyChanged(const std::string& key) override;
  void processMessage(
      const automsgs::msgs::sensor_msgs::RadarTracks& message) override;
  void clearReceivedData() override;
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  struct TrackVisual {
    QVector3D center;
    QVector3D size;
    QVector3D velocity_tip;
    QColor color;
    QColor label_color;
    std::string label;
    bool draw_velocity{false};
  };

  std::vector<TrackVisual> tracks_;
};

}  // namespace display
}  // namespace autoviz
