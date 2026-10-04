/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file range_display.hpp
 * @brief Channel display for @c sensor_msgs/Range (ultrasonic / IR cones).
 *
 * Draws one or more range samples as cones/rays from the sensor origin in the
 * fixed frame — aligned with RViz2 Range.
 *
 * ## Properties
 *
 * - @c color / @c alpha — cone appearance
 * - @c buffer_length — number of historical samples kept in @c history_
 *
 * @see ChannelDisplay
 * @see LaserScanDisplay
 * @see primitive_mesh.hpp
 */

#pragma once

#include <deque>

#include <QVector3D>

#include <automsgs/msgs/sensor_msgs/range.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/channel_display.hpp"

namespace autoviz {
namespace display {

/**
 * @class RangeDisplay
 * @brief Subscribes to Range and draws FOV cones for buffered samples.
 *
 * ## Data flow
 *
 * - **In:** @ref processMessage() TF-transforms the sensor origin and stores
 *   range + field_of_view in @c history_.
 * - **Out:** @ref onDraw() renders each @ref Sample as a cone / sector.
 */
class RangeDisplay
    : public ChannelDisplay<automsgs::msgs::sensor_msgs::Range> {
 public:
  /**
   * @brief Constructs a Range display bound to @p channel.
   *
   * @param channel Autolink / topic name for @c sensor_msgs/Range.
   */
  explicit RangeDisplay(std::string channel);

  /**
   * @brief Catalog type id used by the display registry.
   * @return Always @c "Range".
   */
  std::string typeId() const override { return "Range"; }

  /**
   * @brief Declares color, alpha, and buffer-length properties.
   * @return Property specification list for the Displays tree.
   */
  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  /**
   * @brief Appends a TF-transformed range sample to @c history_.
   *
   * @param message Incoming @c sensor_msgs/Range.
   */
  void processMessage(const automsgs::msgs::sensor_msgs::Range& message)
      override;

  /**
   * @brief Clears @c history_ (Time-panel Reset).
   */
  void clearReceivedData() override;

  /**
   * @brief Draws buffered range samples into @p scene.
   *
   * @param scene Scene overlay / Ogre host for the current frame.
   */
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  /**
   * @struct Sample
   * @brief One range reading in the fixed frame.
   */
  struct Sample {
    QVector3D origin;         /**< Sensor origin (fixed frame). */
    float range = 0.f;        /**< Reported range (meters). */
    float field_of_view = 0.f; /**< Sensor FOV (radians). */
  };

  std::deque<Sample> history_; /**< Ring buffer of recent samples. */
};

}  // namespace display
}  // namespace autoviz
