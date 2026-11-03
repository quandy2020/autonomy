/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/plugins/displays/pose/tf_trajectory_display.hpp"

#include <algorithm>

#include "autoviz/plugins/displays/common/ogre_overlay_draw.hpp"
#include "autoviz/plugins/displays/common/display_tf_utils.hpp"

namespace autoviz {
namespace display {

TFTrajectoryDisplay::TFTrajectoryDisplay() { setProperties({}); }

std::vector<common::DisplayPropertySpec>
TFTrajectoryDisplay::propertySpecs() const {
  return {{"frame", "Frame", "base_link"},
          {"duration", "Duration", "10.0"},
          {"line_color", "Line Color", "31;119;180", {},
           common::DisplayPropertyKind::kColor},
          {"alpha", "Alpha", "1.0"}};
}

void TFTrajectoryDisplay::reset() {
  Display::reset();
  trail_.clear();
}

void TFTrajectoryDisplay::onUpdate() {
  if (context_ == nullptr || context_->tf_buffer == nullptr) {
    setStatusWarn("TF buffer not ready");
    return;
  }
  const std::string frame = propertyValue("frame", "base_link");
  if (frame.empty()) {
    setStatusWarn("Frame not set");
    return;
  }
  automsgs::msgs::geometry_msgs::TransformStamped tf;
  if (!viz::LookupFixedFrameTransform(context_, frame, &tf)) {
    setStatusWarn("Transform unavailable for " + frame);
    return;
  }
  setStatusOk();

  const QVector3D origin(
      static_cast<float>(tf.transform().translation().x()),
      static_cast<float>(tf.transform().translation().y()),
      static_cast<float>(tf.transform().translation().z()));
  const auto now = std::chrono::steady_clock::now();
  trail_.push_back({origin, now});

  const float duration =
      common::ParseFloatProperty(propertyValue("duration", "10.0"), 10.f);
  const auto max_age = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(std::max(0.1, static_cast<double>(duration))));
  while (!trail_.empty() && now - trail_.front().stamp > max_age) {
    trail_.pop_front();
  }
  if (context_->request_redraw) {
    context_->request_redraw();
  }
}

void TFTrajectoryDisplay::onDraw(rendering::SceneOverlay& scene) {
  if (trail_.size() < 2) {
    return;
  }
  QColor color = common::ParseColorProperty(
      propertyValue("line_color", "31;119;180"), QColor(31, 119, 180));
  const float alpha =
      common::ParseFloatProperty(propertyValue("alpha", "1.0"), 1.f);
  color.setAlphaF(alpha);

  std::vector<LineSegment3D> lines;
  lines.reserve(trail_.size() - 1);
  for (std::size_t i = 1; i < trail_.size(); ++i) {
    lines.push_back({trail_[i - 1].position, trail_[i].position, color});
  }
  drawLineSegmentsOgreOrGl(context_, scene, name() + "/tf_trail", lines);
}

}  // namespace display
}  // namespace autoviz
