/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/strata_canvas_label_display.hpp"

#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/commsgs/time_utils.hpp"
#include "autoviz/display/ogre_label_draw.hpp"
#include "autoviz/display/transform_utils.hpp"

namespace autoviz {
namespace display {
namespace {

QColor ProtoColor(const automsgs::msgs::std_msgs::ColorRGBA& color) {
  return QColor(static_cast<int>(color.r() * 255.f),
                static_cast<int>(color.g() * 255.f),
                static_cast<int>(color.b() * 255.f),
                static_cast<int>(color.a() * 255.f));
}

}  // namespace

StrataCanvasLabelDisplay::StrataCanvasLabelDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::strata_msgs::CanvasLabelArray>(
          "StrataCanvasLabel", std::move(channel),
          "automsgs.msgs.strata_msgs.CanvasLabelArray") {
  setProperties({});
}

std::vector<common::DisplayPropertySpec> StrataCanvasLabelDisplay::propertySpecs()
    const {
  using common::DisplayPropertyKind;
  return {{"text_color", "Text Color", "255;255;255", {}, DisplayPropertyKind::kColor},
          {"halo_color", "Halo Color", "30;41;59", {}, DisplayPropertyKind::kColor},
          {"use_message_colors", "Use Message Colors", "true"},
          {"label_height", "Label Height", "0.1"}};
}

void StrataCanvasLabelDisplay::onPropertyChanged(const std::string& /*key*/) {
  if (context_ != nullptr && context_->request_redraw) {
    context_->request_redraw();
  }
}

void StrataCanvasLabelDisplay::processMessage(
    const automsgs::msgs::strata_msgs::CanvasLabelArray& message) {
  labels_.clear();
  if (context_ == nullptr) {
    return;
  }

  const auto zero_time = autoviz::commsgs::ZeroTime();
  const std::string frame = message.header().frame_id().empty()
                                ? context_->fixed_frame
                                : message.header().frame_id();
  const QColor default_color =
      message.has_style() ? ProtoColor(message.style().text_color()) : QColor(255, 255, 255);
  text_color_ = default_color;
  halo_color_ = message.has_style() ? ProtoColor(message.style().halo_color())
                                      : QColor(30, 41, 59);
  halo_blur_ = message.has_style() ? message.style().halo_blur() : 4.f;

  for (const auto& label : message.labels()) {
    if (!label.visible()) {
      continue;
    }
    QVector3D local(static_cast<float>(label.position().x()),
                    static_cast<float>(label.position().y()),
                    static_cast<float>(label.position().z()));
    if (frame != context_->fixed_frame) {
      try {
        const auto tf = context_->tf_buffer->lookupTransform(
            context_->fixed_frame, frame, zero_time);
        local = transformPoint(tf, local);
      } catch (...) {
        continue;
      }
    }
    StoredLabel stored;
    stored.position = local;
    stored.color = default_color;
    stored.halo_color = halo_color_;
    stored.halo_blur = halo_blur_;
    stored.text = QString::fromStdString(label.label());
    labels_.push_back(std::move(stored));
  }

  if (context_->request_redraw) {
    context_->request_redraw();
  }
}

void StrataCanvasLabelDisplay::onDraw(rendering::SceneOverlay& scene) {
  const float label_height =
      common::ParseFloatProperty(propertyValue("label_height", "0.1"), 0.1f);
  const bool use_message_colors =
      common::ParseBoolProperty(propertyValue("use_message_colors", "true"), true);
  const QColor custom_text_color = common::ParseColorProperty(
      propertyValue("text_color", "255;255;255"), QColor(255, 255, 255));
  const QColor custom_halo_color = common::ParseColorProperty(
      propertyValue("halo_color", "30;41;59"), QColor(30, 41, 59));
  const QColor text_color = use_message_colors ? text_color_ : custom_text_color;
  const QColor halo_color = use_message_colors ? halo_color_ : custom_halo_color;
  std::vector<TextLabelInstance> text_labels;
  text_labels.reserve(labels_.size() * 2);
  for (const auto& label : labels_) {
    if (label.halo_blur > 0.f) {
      TextLabelInstance halo;
      halo.text = label.text.toStdString();
      halo.position = label.position + QVector3D(0.f, 0.f, 0.04f);
      halo.color = halo_color;
      halo.char_height = label_height * 1.05f;
      text_labels.push_back(std::move(halo));
    }
    TextLabelInstance instance;
    instance.text = label.text.toStdString();
    instance.position = label.position + QVector3D(0.f, 0.f, 0.05f);
    instance.color = text_color;
    instance.char_height = label_height;
    text_labels.push_back(std::move(instance));
  }
  if (!text_labels.empty()) {
    drawLabelsOgreOrGl(context_, scene, name() + "/labels", text_labels);
  }
}

}  // namespace display
}  // namespace autoviz
