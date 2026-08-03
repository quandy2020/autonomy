/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/strata_iot_bubble_display.hpp"

#include <chrono>

#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/commsgs/time_utils.hpp"
#include "autoviz/display/ogre_colored_points_draw.hpp"
#include "autoviz/display/ogre_label_draw.hpp"
#include "autoviz/display/transform_utils.hpp"

namespace autoviz {
namespace display {
namespace {

QColor EventColor(const std::string& event_type) {
  if (event_type == "warning") {
    return QColor(255, 149, 0);
  }
  if (event_type == "error") {
    return QColor(255, 59, 48);
  }
  if (event_type.find("elevator") != std::string::npos) {
    return QColor(88, 86, 214);
  }
  if (event_type.find("door") != std::string::npos) {
    return QColor(52, 199, 89);
  }
  return QColor(255, 77, 77);
}

}  // namespace

StrataIotBubbleDisplay::StrataIotBubbleDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::strata_msgs::IotBubbleArray>(
          "StrataIotBubble", std::move(channel),
          "automsgs.msgs.strata_msgs.IotBubbleArray") {
  setProperties({});
}

std::vector<common::DisplayPropertySpec> StrataIotBubbleDisplay::propertySpecs()
    const {
  using common::DisplayPropertyKind;
  return {{"color", "Color", "255;77;77", {}, DisplayPropertyKind::kColor},
          {"use_event_color", "Use Event Color", "true"},
          {"label_height", "Label Height", "0.08"}};
}

void StrataIotBubbleDisplay::onPropertyChanged(const std::string& /*key*/) {
  if (context_ != nullptr && context_->request_redraw) {
    context_->request_redraw();
  }
}

void StrataIotBubbleDisplay::processMessage(
    const automsgs::msgs::strata_msgs::IotBubbleArray& message) {
  bubbles_.clear();
  if (context_ == nullptr) {
    return;
  }

  const auto zero_time = autoviz::commsgs::ZeroTime();
  const int64_t now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                             std::chrono::system_clock::now().time_since_epoch())
                             .count();
  const std::string frame = message.header().frame_id().empty()
                                ? context_->fixed_frame
                                : message.header().frame_id();

  for (const auto& bubble : message.bubbles()) {
    if (!bubble.visible()) {
      continue;
    }
    if (bubble.expire_at_ms() > 0 && bubble.expire_at_ms() < now_ms) {
      continue;
    }
    QVector3D local(static_cast<float>(bubble.position().x()),
                    static_cast<float>(bubble.position().y()),
                    static_cast<float>(bubble.position().z()));
    if (frame != context_->fixed_frame) {
      try {
        const auto tf = context_->tf_buffer->lookupTransform(
            context_->fixed_frame, frame, zero_time);
        local = transformPoint(tf, local);
      } catch (...) {
        continue;
      }
    }
    StoredBubble stored;
    *stored.mutable_position() = local;
    stored.event_type = bubble.event_type();
    stored.message = QString::fromStdString(bubble.message());
    bubbles_.push_back(std::move(stored));
  }

  if (context_->request_redraw) {
    context_->request_redraw();
  }
}

void StrataIotBubbleDisplay::onDraw(rendering::SceneOverlay& scene) {
  const bool use_event_color =
      common::ParseBoolProperty(propertyValue("use_event_color", "true"), true);
  const QColor custom_color = common::ParseColorProperty(
      propertyValue("color", "255;77;77"), QColor(255, 77, 77));
  const float label_height =
      common::ParseFloatProperty(propertyValue("label_height", "0.08"), 0.08f);
  std::vector<TextLabelInstance> labels;
  labels.reserve(bubbles_.size());
  for (size_t i = 0; i < bubbles_.size(); ++i) {
    const auto& bubble = bubbles_[i];
    const QColor bubble_color =
        use_event_color ? EventColor(bubble.event_type) : custom_color;
    drawColoredPointsOgreOrGl(context_, scene, name() + "/iot/" + std::to_string(i), typeId(),
                              6.f, rendering::PointCloudStyle::kSquares,
                              {{bubble.position(), bubble_color}}, false);
    if (bubble.message.isEmpty()) {
      continue;
    }
    TextLabelInstance label;
    label.text = bubble.message.toStdString();
    *label.mutable_position() = bubble.position + QVector3D(0.f, 0.f, 0.1f);
    *label.mutable_color() = bubble_color;
    label.char_height = label_height;
    labels.push_back(std::move(label));
  }
  if (!labels.empty()) {
    drawLabelsOgreOrGl(context_, scene, name() + "/labels", labels);
  }
}

}  // namespace display
}  // namespace autoviz
