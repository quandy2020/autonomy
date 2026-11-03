/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/plugins/displays/pictogram/pictogram_display.hpp"

#include <utility>

#include "autoviz/plugins/displays/common/ogre_label_draw.hpp"
#include "autoviz/plugins/displays/common/display_tf_utils.hpp"

namespace autoviz {
namespace display {
namespace {

QColor ColorFromProto(const automsgs::msgs::std_msgs::ColorRGBA& rgba,
                      const QColor& fallback, float alpha) {
  QColor color = fallback;
  if (rgba.a() > 0.f || rgba.r() > 0.f || rgba.g() > 0.f || rgba.b() > 0.f) {
    color = QColor(static_cast<int>(rgba.r() * 255.f),
                   static_cast<int>(rgba.g() * 255.f),
                   static_cast<int>(rgba.b() * 255.f));
    if (rgba.a() > 0.f) {
      color.setAlphaF(rgba.a());
    }
  }
  color.setAlphaF(alpha * color.alphaF());
  return color;
}

}  // namespace

PictogramDisplay::PictogramDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::visualization_msgs::Pictogram>(
          "Pictogram", std::move(channel),
          "automsgs.msgs.visualization_msgs.Pictogram") {
  setProperties({});
}

std::vector<common::DisplayPropertySpec> PictogramDisplay::propertySpecs()
    const {
  return {{"color", "Color", "255;255;255", {}, common::DisplayPropertyKind::kColor},
          {"alpha", "Alpha", "1.0"},
          {"size", "Size", "0.2"}};
}

void PictogramDisplay::clearReceivedData() { labels_.clear(); }

void PictogramDisplay::processMessage(
    const automsgs::msgs::visualization_msgs::Pictogram& message) {
  labels_.clear();
  if (context_ == nullptr) {
    return;
  }
  if (message.action() == automsgs::msgs::visualization_msgs::Pictogram::DELETE) {
    if (context_->request_redraw) {
      context_->request_redraw();
    }
    return;
  }

  automsgs::msgs::geometry_msgs::TransformStamped tf;
  const bool have_tf = viz::LookupFixedFrameTransform(
      context_, message.header().frame_id(), &tf);
  const QVector3D local(static_cast<float>(message.pose().position().x()),
                        static_cast<float>(message.pose().position().y()),
                        static_cast<float>(message.pose().position().z()));

  QColor fallback = common::ParseColorProperty(
      propertyValue("color", "255;255;255"), QColor(255, 255, 255));
  const float alpha =
      common::ParseFloatProperty(propertyValue("alpha", "1.0"), 1.f);
  const float prop_size =
      common::ParseFloatProperty(propertyValue("size", "0.2"), 0.2f);
  const float char_height =
      message.size() > 0.0 ? static_cast<float>(message.size()) : prop_size;

  LabelVisual visual;
  visual.text = message.character();
  if (visual.text.empty() &&
      message.mode() ==
          automsgs::msgs::visualization_msgs::Pictogram::PICTOGRAM_MODE) {
    visual.text = "?";
  }
  visual.position = viz::TransformLocalPoint(tf, have_tf, local);
  visual.color = ColorFromProto(message.color(), fallback, alpha);
  visual.char_height = char_height;
  labels_.push_back(visual);

  if (context_->request_redraw) {
    context_->request_redraw();
  }
}

void PictogramDisplay::onDraw(rendering::SceneOverlay& scene) {
  std::vector<TextLabelInstance> instances;
  instances.reserve(labels_.size());
  for (const LabelVisual& label : labels_) {
    if (label.text.empty()) {
      continue;
    }
    instances.push_back({label.text, label.position, label.color,
                         label.char_height});
  }
  drawLabelsOgreOrGl(context_, scene, name() + "/pictogram", instances);
}

}  // namespace display
}  // namespace autoviz
