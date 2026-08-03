/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/strata_label_bubble_display.hpp"

#include <QRegularExpression>

#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/commsgs/time_utils.hpp"
#include "autoviz/display/ogre_label_draw.hpp"
#include "autoviz/display/screen_projection.hpp"
#include "autoviz/display/transform_utils.hpp"

namespace autoviz {
namespace display {
namespace {

QString StripHtml(const std::string& html) {
  QString text = QString::fromStdString(html);
  text.remove(QRegularExpression("<[^>]*>"));
  return text.trimmed();
}

}  // namespace

StrataLabelBubbleDisplay::StrataLabelBubbleDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::strata_msgs::LabelBubbleArray>(
          "StrataLabelBubble", std::move(channel),
          "automsgs.msgs.strata_msgs.LabelBubbleArray") {
  setProperties({});
}

std::vector<common::DisplayPropertySpec> StrataLabelBubbleDisplay::propertySpecs()
    const {
  using common::DisplayPropertyKind;
  return {{"text_color", "Text Color", "30;41;59", {}, DisplayPropertyKind::kColor},
          {"label_height", "Label Height", "0.09"}};
}

void StrataLabelBubbleDisplay::onPropertyChanged(const std::string& /*key*/) {
  if (context_ != nullptr && context_->request_redraw) {
    context_->request_redraw();
  }
}

void StrataLabelBubbleDisplay::processMessage(
    const automsgs::msgs::strata_msgs::LabelBubbleArray& message) {
  bubbles_.clear();
  if (context_ == nullptr) {
    return;
  }

  const auto zero_time = autoviz::commsgs::ZeroTime();
  const std::string frame = message.header().frame_id().empty()
                                ? context_->fixed_frame
                                : message.header().frame_id();

  for (const auto& bubble : message.bubbles()) {
    if (!bubble.visible()) {
      continue;
    }
    QVector3D local(static_cast<float>(bubble.lng_lat().x()),
                    static_cast<float>(bubble.lng_lat().y()),
                    static_cast<float>(bubble.lng_lat().z()));
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
    stored.anchor = local;
    stored.offset_x = bubble.offset_x();
    stored.offset_y = bubble.offset_y();
    stored.text = StripHtml(bubble.html());
    if (stored.text.isEmpty()) {
      continue;
    }
    bubbles_.push_back(std::move(stored));
  }

  if (context_->request_redraw) {
    context_->request_redraw();
  }
}

void StrataLabelBubbleDisplay::onDraw(rendering::SceneOverlay& scene) {
  const QColor text_color = common::ParseColorProperty(
      propertyValue("text_color", "30;41;59"), QColor(30, 41, 59));
  const float label_height =
      common::ParseFloatProperty(propertyValue("label_height", "0.09"), 0.09f);
  std::vector<TextLabelInstance> labels;
  labels.reserve(bubbles_.size());
  for (const auto& bubble : bubbles_) {
    const QVector3D position =
        applyScreenOffset(context_, bubble.anchor, bubble.offset_x, bubble.offset_y);
    TextLabelInstance label;
    label.text = bubble.text.toStdString();
    *label.mutable_position() = position + QVector3D(0.f, 0.f, 0.08f);
    *label.mutable_color() = text_color;
    label.char_height = label_height;
    labels.push_back(std::move(label));
  }
  if (!labels.empty()) {
    drawLabelsOgreOrGl(context_, scene, name() + "/bubbles", labels);
  }
}

}  // namespace display
}  // namespace autoviz
