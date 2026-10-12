/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_default_plugins FluidPressureDisplay
 * (Willow Garage / TNG Technology Consulting GmbH / Open Source Robotics
 * Foundation).
 *****************************************************************************/

/**
 * @file fluid_pressure_display.hpp
 * @brief Channel display for @c sensor_msgs/FluidPressure (RViz FluidPressure).
 *
 * RViz implements this via @c PointCloudScalarDisplay (single PointCloud2 point
 * with intensity = fluid_pressure, Min/Max 98000–105000). Autoviz draws the
 * same scalar at the message frame origin as a colored crosshair — no ROS /
 * PointCloudCommon.
 *
 * @see ChannelDisplay
 * @see TemperatureDisplay
 * @see RelativeHumidityDisplay
 * @see colorFromScalar()
 */

#pragma once

#include <QVector3D>

#include <automsgs/msgs/sensor_msgs/fluid_pressure.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/plugins/displays/channel_display.hpp"

namespace autoviz {
namespace display {

/**
 * @class FluidPressureDisplay
 * @brief Subscribes to FluidPressure and draws a rainbow-colored point at the
 *        sensor frame origin.
 *
 * ## Properties (RViz-aligned defaults)
 *
 * | Key         | Label       | Default |
 * |-------------|-------------|---------|
 * | min_value   | Min Value   | 98000   |
 * | max_value   | Max Value   | 105000  |
 * | point_size  | Point Size  | 0.12    |
 *
 * Coloring uses intensity rainbow (Invert Rainbow left false, matching RViz).
 */
class FluidPressureDisplay
    : public ChannelDisplay<automsgs::msgs::sensor_msgs::FluidPressure> {
 public:
  /**
   * @brief Constructs a FluidPressure display bound to @p channel.
   *
   * @param channel Autolink channel for @c sensor_msgs/FluidPressure.
   */
  explicit FluidPressureDisplay(std::string channel);

  /**
   * @brief Catalog type id.
   * @return Always @c "FluidPressure".
   */
  std::string typeId() const override { return "FluidPressure"; }

  /**
   * @brief Declares min/max value and point-size properties.
   * @return Specs for the Displays property editor.
   */
  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  /**
   * @brief Caches fluid pressure and TF origin from @p message.
   *
   * @param message Incoming @c sensor_msgs/FluidPressure protobuf.
   */
  void processMessage(
      const automsgs::msgs::sensor_msgs::FluidPressure& message) override;

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
  double value_ = 0.0;       /**< Latest fluid pressure (Pa). */
  QVector3D position_;       /**< Sensor origin in the fixed frame. */
};

}  // namespace display
}  // namespace autoviz
