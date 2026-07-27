/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

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

/** rviz-style scalar sensor: single colored point at sensor origin. */
template <typename MessageT>
class ScalarSensorDisplay : public ChannelDisplay<MessageT> {
 public:
  using ValueFn = std::function<double(const MessageT&)>;

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

  std::vector<common::DisplayPropertySpec> propertySpecs() const override {
    return {{"min_value", "Min Value", std::to_string(default_min_)},
            {"max_value", "Max Value", std::to_string(default_max_)},
            {"point_size", "Point Size", "0.12"}};
  }

 protected:
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
  ValueFn value_fn_;
  std::string min_key_;
  double default_min_;
  std::string max_key_;
  double default_max_;
  bool have_sample_ = false;
  double value_ = 0.0;
  QVector3D position_;
};

}  // namespace display
}  // namespace autoviz
