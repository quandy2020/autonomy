/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file laser_scan_display.hpp
 * @brief Displays @c sensor_msgs/LaserScan as a colored 2D/3D point set
 *        (RViz LaserScan).
 *
 * Converts polar ranges to Cartesian points in the scan frame, optionally
 * colors by intensity, and draws with configurable point size/style.
 *
 * @see ChannelDisplay
 * @see DepthCloudDisplay
 */

#pragma once

#include <QColor>
#include <QVector3D>
#include <vector>

#include <automsgs/msgs/sensor_msgs/laser_scan.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/channel_display.hpp"

namespace autoviz {
namespace display {

/**
 * @class LaserScanDisplay
 * @brief Channel display for LaserScan point clouds.
 *
 * Properties typically include alpha, flat color / intensity transform,
 * point size, and style (squares, etc.).
 *
 * @see ChannelDisplay
 */
class LaserScanDisplay
    : public ChannelDisplay<automsgs::msgs::sensor_msgs::LaserScan> {
 public:
  /**
   * @brief Constructs the display bound to @p channel.
   *
   * @param channel Autolink channel for LaserScan messages.
   */
  explicit LaserScanDisplay(std::string channel);

  /**
   * @brief Catalog type id.
   *
   * @return Always @c "LaserScan".
   */
  std::string typeId() const override { return "LaserScan"; }

  /**
   * @brief Property schema (color, intensity, size, style, …).
   *
   * @return Specs for the Displays property editor.
   */
  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  /**
   * @brief Converts ranges into @c points_ with per-point colors.
   *
   * @param message sensor_msgs/LaserScan protobuf.
   */
  void processMessage(
      const automsgs::msgs::sensor_msgs::LaserScan& message) override;

  /**
   * @brief Draws @c points_ into @p scene.
   *
   * @param scene Scene overlay destination.
   */
  void onDraw(rendering::SceneOverlay& scene) override;

  /**
   * @brief Clears @c points_.
   */
  void clearReceivedData() override;

 private:
  /**
   * @struct ScanPoint
   * @brief One laser return in Cartesian coordinates with display color.
   */
  struct ScanPoint {
    QVector3D position;  /**< Point in scan / fixed frame as processed. */
    QColor color;        /**< Flat or intensity-mapped RGBA. */
  };

  /** Cached points from the latest scan. */
  std::vector<ScanPoint> points_;
};

}  // namespace display
}  // namespace autoviz
