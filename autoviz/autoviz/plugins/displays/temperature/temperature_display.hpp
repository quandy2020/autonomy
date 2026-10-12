/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_default_plugins TemperatureDisplay
 * (Willow Garage / TNG Technology Consulting GmbH / Open Source Robotics
 * Foundation).
 *****************************************************************************/

/**
 * @file temperature_display.hpp
 * @brief Channel display for @c sensor_msgs/Temperature (RViz Temperature).
 *
 * RViz implements this via @c PointCloudScalarDisplay (single PointCloud2 point
 * with intensity = temperature, Invert Rainbow, Min/Max 0–100). Autoviz draws
 * the same scalar at the message frame origin as a colored crosshair — no ROS /
 * PointCloudCommon.
 *
 * @see ChannelDisplay
 * @see ScalarSensorDisplay
 * @see colorFromScalar()
 */

#pragma once

#include <QVector3D>

#include <automsgs/msgs/sensor_msgs/temperature.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/plugins/displays/channel_display.hpp"

namespace autoviz {
namespace display {

/**
 * @class TemperatureDisplay
 * @brief Subscribes to Temperature and draws a rainbow-colored point at the
 *        sensor frame origin.
 *
 * ## Properties (RViz-aligned defaults)
 *
 * | Key         | Label       | Default |
 * |-------------|-------------|---------|
 * | min_value   | Min Value   | 0       |
 * | max_value   | Max Value   | 100     |
 * | point_size  | Point Size  | 0.12    |
 *
 * Coloring uses intensity rainbow with Invert Rainbow = true (RViz default).
 */
class TemperatureDisplay
    : public ChannelDisplay<automsgs::msgs::sensor_msgs::Temperature> {
 public:
  /**
   * @brief Constructs a Temperature display bound to @p channel.
   *
   * @param channel Autolink channel for @c sensor_msgs/Temperature.
   */
  explicit TemperatureDisplay(std::string channel);

  /**
   * @brief Catalog type id.
   * @return Always @c "Temperature".
   */
  std::string typeId() const override { return "Temperature"; }

  /**
   * @brief Declares min/max value and point-size properties.
   * @return Specs for the Displays property editor.
   */
  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  /**
   * @brief Caches temperature and TF origin from @p message.
   *
   * @param message Incoming @c sensor_msgs/Temperature protobuf.
   */
  void processMessage(
      const automsgs::msgs::sensor_msgs::Temperature& message) override;

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
  double value_ = 0.0;       /**< Latest temperature (°C / message units). */
  QVector3D position_;       /**< Sensor origin in the fixed frame. */
};

}  // namespace display
}  // namespace autoviz
