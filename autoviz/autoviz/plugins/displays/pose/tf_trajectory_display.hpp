/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <chrono>
#include <deque>
#include <string>
#include <vector>

#include <QColor>
#include <QVector3D>

#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/plugins/displays/channel_display.hpp"

namespace autoviz {
namespace display {

/**
 * @class TFTrajectoryDisplay
 * @brief Sci-fi comet motion trail (残影光轨).
 *
 * Prefer @c /odom samples when a channel is set (reliable). Also densifies from
 * TF (@c frame, with base_link / base_footprint fallbacks) each update tick.
 */
class TFTrajectoryDisplay
    : public ChannelDisplay<automsgs::msgs::nav_msgs::Odometry> {
 public:
  explicit TFTrajectoryDisplay(std::string channel = "");

  std::string typeId() const override { return "TFTrajectory"; }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  void processMessage(
      const automsgs::msgs::nav_msgs::Odometry& message) override;
  void clearReceivedData() override;
  void onUpdate() override;
  void onDraw(rendering::SceneOverlay& scene) override;
  void reset() override;

 private:
  struct TrailSample {
    QVector3D position;
    std::chrono::steady_clock::time_point stamp;
  };

  void appendSample(const QVector3D& position,
                    std::chrono::steady_clock::time_point now);
  void pruneExpired(std::chrono::steady_clock::time_point now);
  bool tryAppendFromTf(std::chrono::steady_clock::time_point now);
  void clearLayers(rendering::SceneOverlay& scene);

  std::deque<TrailSample> trail_;
  bool layers_drawn_ = false;
};

}  // namespace display
}  // namespace autoviz
