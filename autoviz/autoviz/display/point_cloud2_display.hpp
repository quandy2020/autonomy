/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file point_cloud2_display.hpp
 * @brief Channel display for @c sensor_msgs/PointCloud2.
 *
 * Parses clouds via @ref parsePointCloud2, applies RViz2-style color
 * transformers, optional decay, and draws through
 * @ref drawColoredPointsOgreOrGl.
 *
 * ## Properties
 *
 * Mirrors @c rviz_default_plugins PointCloudCommon + transformers:
 * selectable, style, size (m / pixels), alpha, decay time, position /
 * color transformers, channel, min/max, rainbow options.
 *
 * @see PointCloud2Display
 * @see point_cloud_utils.hpp
 * @see ogre_colored_points_draw.hpp
 * @see LaserScanDisplay
 */

#pragma once

#include <chrono>
#include <QColor>
#include <QVector3D>
#include <vector>

#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/channel_display.hpp"

namespace autoviz {
namespace display {

/**
 * @class PointCloud2Display
 * @brief Subscribes to PointCloud2 and renders colored point batches.
 *
 * ## Data flow
 *
 * - **In:** @ref processMessage() decodes fields, colors points, and appends
 *   a @ref PointBatch (evicting by @c decay_time).
 * - **Out:** @ref onDraw() flattens live batches into @ref ColoredPoint3D and
 *   calls @ref drawColoredPointsOgreOrGl.
 *
 * @note When @c decay_time is @c 0 only the newest batch is kept.
 */
class PointCloud2Display
    : public ChannelDisplay<automsgs::msgs::sensor_msgs::PointCloud2> {
 public:
  /**
   * @brief Constructs a PointCloud2 display bound to @p channel.
   *
   * @param channel Autolink / topic name for @c sensor_msgs/PointCloud2.
   */
  explicit PointCloud2Display(std::string channel);

  /**
   * @brief Catalog type id used by the display registry.
   * @return Always @c "PointCloud2".
   */
  std::string typeId() const override { return "PointCloud2"; }

  /**
   * @brief Declares PointCloudCommon + transformer property specs.
   * @return Property specification list for the Displays tree.
   */
  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  /**
   * @brief Parses, colors, and buffers a new cloud message.
   *
   * @param message Incoming @c sensor_msgs/PointCloud2.
   */
  void processMessage(
      const automsgs::msgs::sensor_msgs::PointCloud2& message)
      override;

  /**
   * @brief Draws all non-expired point batches into @p scene.
   *
   * @param scene Scene overlay / Ogre host for the current frame.
   */
  void onDraw(rendering::SceneOverlay& scene) override;

  /**
   * @brief Clears @c batches_ (Time-panel Reset).
   */
  void clearReceivedData() override;

 private:
  /**
   * @struct CloudPoint
   * @brief One colored point with receive timestamp for decay eviction.
   */
  struct CloudPoint {
    QVector3D position; /**< Fixed-frame position. */
    QColor color;       /**< Transformer output color. */
    /** Wall-clock time when this point was received (for decay eviction). */
    std::chrono::steady_clock::time_point received_at;
  };

  /**
   * @struct PointBatch
   * @brief Points from one message, keyed by receive time.
   *
   * Buckets of points indexed by message sequence (newest appended at back).
   * When @c decay_time == 0 only the most-recent batch is kept.
   */
  struct PointBatch {
    std::vector<CloudPoint> points; /**< Points from one cloud message. */
    std::chrono::steady_clock::time_point received_at; /**< Batch receive time. */
  };

  std::vector<PointBatch> batches_; /**< Live point history for decay. */
};

}  // namespace display
}  // namespace autoviz
