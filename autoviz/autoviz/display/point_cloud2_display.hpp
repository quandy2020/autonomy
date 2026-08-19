/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <chrono>
#include <limits>
#include <QColor>
#include <QVector3D>
#include <vector>

#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/channel_display.hpp"

namespace autoviz {
namespace display {

class PointCloud2Display
    : public ChannelDisplay<automsgs::msgs::sensor_msgs::PointCloud2> {
 public:
  explicit PointCloud2Display(std::string channel);

  std::string typeId() const override { return "PointCloud2"; }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  void processMessage(
      const automsgs::msgs::sensor_msgs::PointCloud2& message)
      override;
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  struct CloudPoint {
    QVector3D position;
    QColor color;
    /// Wall-clock time when this point was received (for decay eviction).
    std::chrono::steady_clock::time_point received_at;
  };

  /// Buckets of points indexed by message sequence (newest appended at back).
  /// When decay_time == 0 only the most-recent batch is kept.
  struct PointBatch {
    std::vector<CloudPoint> points;
    std::chrono::steady_clock::time_point received_at;
  };

  std::vector<PointBatch> batches_;

  /// Per-message intensity range remembered across frames for stable coloring.
  float intensity_auto_min_ = std::numeric_limits<float>::max();
  float intensity_auto_max_ = std::numeric_limits<float>::lowest();
};

}  // namespace display
}  // namespace autoviz
