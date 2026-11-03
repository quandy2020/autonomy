/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/plugins/displays/pictogram/pictogram_array_display.hpp"

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

PictogramArrayDisplay::PictogramArrayDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::visualization_msgs::PictogramArray>(
          "PictogramArray", std::move(channel),
          "automsgs.msgs.visualization_msgs.PictogramArray") {
  setProperties({});
}

std::vector<common::DisplayPropertySpec>
PictogramArrayDisplay::propertySpecs() const {
  return {{"color", "Color", "255;255;255", {}, common::DisplayPropertyKind::kColor},
          {"alpha", "Alpha", "1.0"},
          {"size", "Size", "0.2"}};
}

void PictogramArrayDisplay::clearReceivedData() { labels_.clear(); }

void PictogramArrayDisplay::processMessage(
    const automsgs::msgs::visualization_msgs::PictogramArray& message) {
  labels_.clear();
  if (context_ == nullptr) {
    return;
  }
  automsgs::msgs::geometry_msgs::TransformStamped array_tf;
  const bool have_array_tf = viz::LookupFixedFrameTransform(
      context_, message.header().frame_id(), &array_tf);

  QColor fallback = common::ParseColorProperty(
      propertyValue("color", "255;255;255"), QColor(255, 255, 255));
  const float alpha =
      common::ParseFloatProperty(propertyValue("alpha", "1.0"), 1.f);
  const float prop_size =
      common::ParseFloatProperty(propertyValue("size", "0.2"), 0.2f);

  labels_.reserve(static_cast<std::size_t>(message.pictograms_size()));
  for (int i = 0; i < message.pictograms_size(); ++i) {
    const auto& pictogram = message.pictograms(i);
    if (pictogram.action() ==
        automsgs::msgs::visualization_msgs::Pictogram::DELETE) {
      continue;
    }
    automsgs::msgs::geometry_msgs::TransformStamped tf = array_tf;
    bool have_tf = have_array_tf;
    if (!pictogram.header().frame_id().empty()) {
      have_tf = viz::LookupFixedFrameTransform(
          context_, pictogram.header().frame_id(), &tf);
    }
    const QVector3D local(
        static_cast<float>(pictogram.pose().position().x()),
        static_cast<float>(pictogram.pose().position().y()),
        static_cast<float>(pictogram.pose().position().z()));
    LabelVisual visual;
    visual.text = pictogram.character();
    if (visual.text.empty() &&
        pictogram.mode() ==
            automsgs::msgs::visualization_msgs::Pictogram::PICTOGRAM_MODE) {
      visual.text = "?";
    }
    visual.position = viz::TransformLocalPoint(tf, have_tf, local);
    visual.color = ColorFromProto(pictogram.color(), fallback, alpha);
    visual.char_height = pictogram.size() > 0.0
                             ? static_cast<float>(pictogram.size())
                             : prop_size;
    labels_.push_back(visual);
  }
  if (context_->request_redraw) {
    context_->request_redraw();
  }
}

void PictogramArrayDisplay::onDraw(rendering::SceneOverlay& scene) {
  std::vector<TextLabelInstance> instances;
  instances.reserve(labels_.size());
  for (const LabelVisual& label : labels_) {
    if (label.text.empty()) {
      continue;
    }
    instances.push_back({label.text, label.position, label.color,
                         label.char_height});
  }
  drawLabelsOgreOrGl(context_, scene, name() + "/pictogram_array", instances);
}

}  // namespace display
}  // namespace autoviz
