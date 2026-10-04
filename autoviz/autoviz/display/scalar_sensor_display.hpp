/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file scalar_sensor_display.hpp
 * @brief Template channel display for single-valued sensors at the frame origin.
 *
 * RViz-style scalar sensor: extracts a @c double via @ref ValueFn, TF-looks up
 * the message frame origin, and draws a colored crosshair point scaled by
 * @ref colorFromScalar (from @ref scalar_sensor_utils.hpp).
 *
 * Typical instantiations wrap FluidPressure, Illuminance, RelativeHumidity,
 * Temperature, etc.
 *
 * ## Properties
 *
 * - @c min_value / @c max_value — coloring range (defaults from ctor)
 * - @c point_size — crosshair half-extent in meters
 *
 * @note This is a header-only template; coloring uses
 *       @c display::colorFromScalar(double, double, double), not the
 *       PointCloud2 overload in @ref point_cloud_utils.hpp.
 *
 * @see ScalarSensorDisplay
 * @see scalar_sensor_utils.hpp
 * @see ChannelDisplay
 * @see transformPoint()
 */

#pragma once

#include <functional>
#include <string>

#include <QVector3D>

#include "autoviz/commsgs/time_utils.hpp"
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/channel_display.hpp"
#include "autoviz/display/scalar_sensor_utils.hpp"
#include "autoviz/display/transform_utils.hpp"

namespace autoviz {
namespace display {

/**
 * @class ScalarSensorDisplay
 * @brief Generic “colored point at sensor origin” display for scalar messages.
 *
 * @tparam MessageT Protobuf message type with @c header().frame_id().
 *
 * ## Data flow
 *
 * - **In:** @ref processMessage() evaluates @c value_fn_, looks up TF to the
 *   fixed frame, and stores origin + value.
 * - **Out:** @ref onDraw() colors by min/max and draws a point + crosshair.
 */
template <typename MessageT>
class ScalarSensorDisplay : public ChannelDisplay<MessageT> {
 public:
  /**
   * @brief Functor extracting the scalar from @tparam MessageT.
   */
  using ValueFn = std::function<double(const MessageT&)>;

  /**
   * @brief Constructs a scalar sensor display.
   *
   * @param type_id Catalog type id (e.g. @c "Temperature").
   * @param channel Autolink / topic name.
   * @param message_type Wire type string for the channel registry.
   * @param value_fn Extracts the scalar from each message.
   * @param min_key Reserved property key name (historical; UI uses fixed keys).
   * @param default_min Default Min Value.
   * @param max_key Reserved property key name (historical).
   * @param default_max Default Max Value.
   */
  ScalarSensorDisplay(std::string type_id, std::string channel,
                      std::string message_type, ValueFn value_fn,
                      std::string min_key, double default_min,
                      std::string max_key, double default_max)
      : ChannelDisplay<MessageT>(std::move(type_id), std::move(channel),
                                 std::move(message_type)),
        value_fn_(std::move(value_fn)),
        min_key_(std::move(min_key)),
        default_min_(default_min),
        max_key_(std::move(max_key)),
        default_max_(default_max) {
    this->setProperties({});
  }

  /**
   * @brief Declares min/max value and point-size properties.
   * @return Property specification list for the Displays tree.
   */
  std::vector<common::DisplayPropertySpec> propertySpecs() const override {
    return {{"min_value", "Min Value", std::to_string(default_min_)},
            {"max_value", "Max Value", std::to_string(default_max_)},
            {"point_size", "Point Size", "0.12"}};
  }

 protected:
  /**
   * @brief Evaluates the scalar, looks up TF origin, and caches the sample.
   *
   * @param message Incoming sensor message.
   */
  void processMessage(const MessageT& message) override {
    if (this->context_ == nullptr || !value_fn_) {
      return;
    }
    value_ = value_fn_(message);
    const auto zero_time = autoviz::commsgs::ZeroTime();
    const std::string frame = message.header().frame_id().empty()
                                  ? this->context_->fixed_frame
                                  : message.header().frame_id();
    try {
      const auto tf = this->context_->tf_buffer->lookupTransform(
          this->context_->fixed_frame, frame, zero_time);
      position_ = transformPoint(tf, QVector3D(0.f, 0.f, 0.f));
    } catch (...) {
      position_ = QVector3D(0.f, 0.f, 0.f);
    }
    have_sample_ = true;
    if (this->context_->request_redraw) {
      this->context_->request_redraw();
    }
  }

  /**
   * @brief Clears @c have_sample_ (Time-panel Reset).
   */
  void clearReceivedData() override { have_sample_ = false; }

  /**
   * @brief Draws a colored point and axis-aligned crosshair at the origin.
   *
   * @param scene Scene overlay for the current frame.
   */
  void onDraw(rendering::SceneOverlay& scene) override {
    if (!have_sample_) {
      return;
    }
    const double min_value = common::ParseFloatProperty(
        this->propertyValue("min_value", std::to_string(default_min_)),
        static_cast<float>(default_min_));
    const double max_value = common::ParseFloatProperty(
        this->propertyValue("max_value", std::to_string(default_max_)),
        static_cast<float>(default_max_));
    const float point_size = common::ParseFloatProperty(
        this->propertyValue("point_size", "0.12"), 0.12f);
    const QColor color = colorFromScalar(value_, min_value, max_value);
    scene.addPoint(position_, color);
    scene.addLine(position_ - QVector3D(point_size, 0.f, 0.f),
                  position_ + QVector3D(point_size, 0.f, 0.f), color);
    scene.addLine(position_ - QVector3D(0.f, point_size, 0.f),
                  position_ + QVector3D(0.f, point_size, 0.f), color);
  }

 private:
  ValueFn value_fn_;          /**< Scalar extractor. */
  std::string min_key_;       /**< Historical min property key (unused in draw). */
  double default_min_;        /**< Default Min Value. */
  std::string max_key_;       /**< Historical max property key (unused in draw). */
  double default_max_;        /**< Default Max Value. */
  bool have_sample_ = false;  /**< Whether @c value_ / @c position_ are valid. */
  double value_ = 0.0;        /**< Latest scalar. */
  QVector3D position_;        /**< Sensor origin in fixed frame. */
};

}  // namespace display
}  // namespace autoviz
