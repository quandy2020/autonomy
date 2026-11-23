/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file radar_scan_display.hpp
 * @brief RadarScan PPI returns + screen-space range HUD.
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <QColor>
#include <QVector3D>

#include <automsgs/msgs/sensor_msgs/radar.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/plugins/displays/channel_display.hpp"
#include "autoviz/plugins/displays/common/overlay_object.hpp"

namespace autoviz {
namespace display {

/**
 * @class RadarScanDisplay
 * @brief Range returns as glowing spheres with spokes, halos, and a HUD card.
 */
class RadarScanDisplay
    : public ChannelDisplay<automsgs::msgs::sensor_msgs::RadarScan> {
 public:
  explicit RadarScanDisplay(std::string channel);
  ~RadarScanDisplay() override;

  std::string typeId() const override { return "RadarScan"; }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  void onEnable() override;
  void onDisable() override;
  void onPropertyChanged(const std::string& key) override;
  void processMessage(
      const automsgs::msgs::sensor_msgs::RadarScan& message) override;
  void onDraw(rendering::SceneOverlay& scene) override;
  void clearReceivedData() override;

 private:
  struct ScanPoint {
    QVector3D position;
    QColor color;
    float range{0.f};
    float azimuth_deg{0.f};
    float velocity_mps{0.f};
    float amplitude{0.f};
    bool primary{false};
  };

  void ensureHud();
  void refreshHud();
  static void DrawBillboardLabel(rendering::SceneOverlay& scene,
                                 const QVector3D& pos, const QString& text,
                                 const QColor& color, float char_height_m);
  static void DrawHalo(rendering::SceneOverlay& scene, const QVector3D& center,
                       float radius, const QColor& color);

  std::vector<ScanPoint> points_;
  float primary_range_{-1.f};
  float primary_angle_deg_{0.f};
  float primary_velocity_mps_{0.f};
  int return_count_{0};
  bool have_angle_vel_{false};
  bool hud_dirty_{true};
  viz::OverlayObject::Ptr hud_;
};

}  // namespace display
}  // namespace autoviz
