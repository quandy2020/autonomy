/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_default_plugins RelativeHumidityDisplay
 * (Willow Garage / TNG Technology Consulting GmbH / Open Source Robotics
 * Foundation).
 *****************************************************************************/

/**
 * @file relative_humidity_display.hpp
 * @brief Channel display for @c sensor_msgs/RelativeHumidity (RViz).
 *
 * RViz implements this via @c PointCloudScalarDisplay (single PointCloud2 point
 * with intensity = relative_humidity, Min/Max 0–1). Autoviz draws the same
 * scalar at the message frame origin as a colored crosshair — no ROS /
 * PointCloudCommon.
 *
 * @see ChannelDisplay
 * @see TemperatureDisplay
 * @see colorFromScalar()
 */

#pragma once

#include <QVector3D>

#include <automsgs/msgs/sensor_msgs/relative_humidity.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/plugins/displays/channel_display.hpp"

namespace autoviz {
namespace display {

/**
 * @class RelativeHumidityDisplay
 * @brief Subscribes to RelativeHumidity and draws a rainbow-colored point at
 *        the sensor frame origin.
 *
 * ## Properties (RViz-aligned defaults)
 *
 * | Key         | Label       | Default |
 * |-------------|-------------|---------|
 * | min_value   | Min Value   | 0       |
 * | max_value   | Max Value   | 1       |
 * | point_size  | Point Size  | 0.12    |
 *
 * Coloring uses intensity rainbow (Invert Rainbow left false, matching RViz).
 */
class RelativeHumidityDisplay
    : public ChannelDisplay<automsgs::msgs::sensor_msgs::RelativeHumidity> {
 public:
  /**
   * @brief Constructs a RelativeHumidity display bound to @p channel.
   *
   * @param channel Autolink channel for @c sensor_msgs/RelativeHumidity.
   */
  explicit RelativeHumidityDisplay(std::string channel);

  /**
   * @brief Catalog type id.
   * @return Always @c "RelativeHumidity".
   */
  std::string typeId() const override { return "RelativeHumidity"; }

  /**
   * @brief Declares min/max value and point-size properties.
   * @return Specs for the Displays property editor.
   */
  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  /**
   * @brief Caches relative humidity and TF origin from @p message.
   *
   * @param message Incoming @c sensor_msgs/RelativeHumidity protobuf.
   */
  void processMessage(
      const automsgs::msgs::sensor_msgs::RelativeHumidity& message) override;

  /**
   * @brief Clears @c have_sample_ (Time-panel Reset).
   */
  void clearReceivedData() override;

  /**
   * @brief Draws a colored point + crosshair at the sensor origin.
   *
   * @param scene Scene overlay for the current frame.
   */
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  bool have_sample_ = false; /**< Whether @c value_ / @c position_ are valid. */
  double value_ = 0.0;       /**< Latest relative humidity [0, 1]. */
  QVector3D position_;       /**< Sensor origin in the fixed frame. */
};

}  // namespace display
}  // namespace autoviz
