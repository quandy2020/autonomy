/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file point_cloud2_display.hpp
 * @brief Channel display for @c sensor_msgs/PointCloud2.
 *
 * Parses clouds via @ref parsePointCloud2, applies RViz2-style color
 * transformers, optional decay, and uploads through
 * @ref rendering::OgreSceneHost (dirty / incremental — not every UI tick).
 *
 * ## Properties
 *
 * Mirrors @c rviz_default_plugins PointCloudCommon + transformers:
 * selectable, style, size (m / pixels), alpha, decay time, position /
 * color transformers, channel, min/max, rainbow options, max points.
 *
 * @see PointCloud2Display
 * @see point_cloud_utils.hpp
 * @see LaserScanDisplay
 */

#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/plugins/displays/channel_display.hpp"
#include "autoviz/rendering/objects/ogre_point_cloud.hpp"
#include "autoviz/rendering/render_settings.hpp"

namespace autoviz {
namespace display {

/**
 * @class PointCloud2Display
 * @brief Subscribes to PointCloud2 and renders colored point batches.
 *
 * ## Data flow (RViz2-aligned)
 *
 * - **In:** @ref processMessage() colors points into @ref PointBatch and
 *   either replaces (@c decay_time==0) or appends to the Ogre cloud.
 * - **Out:** @ref onDraw() only re-uploads when @c gpu_dirty_; otherwise the
 *   existing GPU buffers stay (camera motion does not rebuild dense clouds).
 *
 * @note When @c decay_time is @c 0 only the newest batch is kept.
 */
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

  void clearReceivedData() override;

  void onDisable() override;

  std::size_t messageQueueDepth() const override;

  void afterQueueProcessed() override;

 private:
  struct PointBatch {
    std::vector<rendering::OgrePointCloud::Point> points;
    std::chrono::steady_clock::time_point received_at;
  };

  /** @brief Drop expired batches; pop matching GPU points when incremental. */
  void evictExpiredBatches();

  /** @brief Resolve style/size; force Points for huge billboard clouds. */
  void resolveDrawStyle(rendering::PointCloudStyle* style,
                        float* point_size, std::size_t total_points) const;

  /** @brief Full GPU replace from @c batches_ (style/size change, reset). */
  void uploadGpuFull();

  /** @brief Append one batch to the Ogre cloud (Decay Time &gt; 0). */
  void appendGpuBatch(std::vector<rendering::OgrePointCloud::Point>&& points);

  std::vector<PointBatch> batches_;
  /** True when Ogre buffers must be rebuilt from @c batches_. */
  bool gpu_dirty_ = false;
  /**
   * When true, GPU matches batches via append/pop (RViz2). Style/size changes
   * clear this and force @ref uploadGpuFull().
   */
  bool gpu_incremental_ok_ = false;
  rendering::PointCloudStyle last_style_ =
      rendering::PointCloudStyle::kPoints;
  float last_point_size_ = -1.f;
};

}  // namespace display
}  // namespace autoviz
