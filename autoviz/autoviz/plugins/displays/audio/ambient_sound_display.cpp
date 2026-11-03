/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/plugins/displays/audio/ambient_sound_display.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include <QtMath>

#include "autoviz/plugins/displays/common/ogre_overlay_draw.hpp"
#include "autoviz/plugins/displays/common/display_tf_utils.hpp"

namespace autoviz {
namespace display {

AmbientSoundDisplay::AmbientSoundDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::visualization_msgs::HarkPower>(
          "AmbientSound", std::move(channel),
          "automsgs.msgs.visualization_msgs.HarkPower") {
  setProperties({});
}

std::vector<common::DisplayPropertySpec>
AmbientSoundDisplay::propertySpecs() const {
  return {{"color", "Color", "23;190;207", {}, common::DisplayPropertyKind::kColor},
          {"alpha", "Alpha", "1.0"},
          {"max_length", "Max Bar Length", "1.0"},
          {"style", "Style", "Arrows", {"Arrows", "Lines"}}};
}

void AmbientSoundDisplay::clearReceivedData() {
  arrows_.clear();
  have_origin_ = false;
}

void AmbientSoundDisplay::processMessage(
    const automsgs::msgs::visualization_msgs::HarkPower& message) {
  arrows_.clear();
  if (context_ == nullptr) {
    return;
  }
  automsgs::msgs::geometry_msgs::TransformStamped tf;
  const bool have_tf = viz::LookupFixedFrameTransform(
      context_, message.header().frame_id(), &tf);
  origin_ = have_tf ? QVector3D(static_cast<float>(tf.transform().translation().x()),
                                static_cast<float>(tf.transform().translation().y()),
                                static_cast<float>(tf.transform().translation().z()))
                    : QVector3D();
  have_origin_ = true;

  QColor color = common::ParseColorProperty(propertyValue("color", "23;190;207"),
                                            QColor(23, 190, 207));
  const float alpha =
      common::ParseFloatProperty(propertyValue("alpha", "1.0"), 1.f);
  color.setAlphaF(alpha);
  const float max_length =
      common::ParseFloatProperty(propertyValue("max_length", "1.0"), 1.f);

  float max_power = 0.f;
  for (int i = 0; i < message.powers_size(); ++i) {
    max_power = std::max(max_power, message.powers(i));
  }
  if (max_power <= 1e-6f) {
    max_power = 1.f;
  }

  const int bins = message.powers_size();
  if (bins <= 0) {
    if (context_->request_redraw) {
      context_->request_redraw();
    }
    return;
  }

  for (int i = 0; i < bins; ++i) {
    const float angle =
        static_cast<float>(2.0 * M_PI * i / bins);
    const float length =
        max_length * (message.powers(i) / max_power);
    const QVector3D dir(qCos(angle), qSin(angle), 0.f);
    arrows_.push_back({origin_, origin_ + dir * length, color});
  }
  if (context_->request_redraw) {
    context_->request_redraw();
  }
}

void AmbientSoundDisplay::onDraw(rendering::SceneOverlay& scene) {
  if (!have_origin_) {
    return;
  }
  const bool use_arrows = propertyValue("style", "Arrows") == "Arrows";
  if (use_arrows) {
    for (std::size_t i = 0; i < arrows_.size(); ++i) {
      const ArrowVisual& arrow = arrows_[i];
      drawArrowOgreOrGl(context_, scene, name() + "/hark/" + std::to_string(i),
                        arrow.start, arrow.end, arrow.color, 0.25f);
    }
    return;
  }
  std::vector<LineSegment3D> lines;
  lines.reserve(arrows_.size());
  for (const ArrowVisual& arrow : arrows_) {
    lines.push_back({arrow.start, arrow.end, arrow.color});
  }
  drawLineSegmentsOgreOrGl(context_, scene, name() + "/hark", lines);
}

}  // namespace display
}  // namespace autoviz
