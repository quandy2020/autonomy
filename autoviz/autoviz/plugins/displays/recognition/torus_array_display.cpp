/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/plugins/displays/recognition/torus_array_display.hpp"

#include <cmath>
#include <utility>

#include <QtMath>

#include "autoviz/plugins/displays/common/ogre_overlay_draw.hpp"
#include "autoviz/plugins/displays/common/color_category.hpp"
#include "autoviz/plugins/displays/common/display_tf_utils.hpp"

namespace autoviz {
namespace display {

TorusArrayDisplay::TorusArrayDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::visualization_msgs::TorusArray>(
          "TorusArray", std::move(channel),
          "automsgs.msgs.visualization_msgs.TorusArray") {
  setProperties({});
}

std::vector<common::DisplayPropertySpec>
TorusArrayDisplay::propertySpecs() const {
  return {{"coloring", "Coloring", "Auto", {"Auto", "Flat"}},
          {"color", "Color", "25;255;240", {},
           common::DisplayPropertyKind::kColor},
          {"alpha", "Alpha", "1.0"},
          {"segments", "Ring Segments", "32"}};
}

void TorusArrayDisplay::clearReceivedData() { segments_.clear(); }

void TorusArrayDisplay::processMessage(
    const automsgs::msgs::visualization_msgs::TorusArray& message) {
  segments_.clear();
  if (context_ == nullptr) {
    return;
  }
  automsgs::msgs::geometry_msgs::TransformStamped tf;
  const bool have_tf = viz::LookupFixedFrameTransform(
      context_, message.header().frame_id(), &tf);

  const bool auto_color = propertyValue("coloring", "Auto") == "Auto";
  QColor flat = common::ParseColorProperty(
      propertyValue("color", "25;255;240"), QColor(25, 255, 240));
  const float alpha =
      common::ParseFloatProperty(propertyValue("alpha", "1.0"), 1.f);
  flat.setAlphaF(alpha);
  const int ring_segments = std::max(
      8, common::ParseIntProperty(propertyValue("segments", "32"), 32));

  for (int i = 0; i < message.toruses_size(); ++i) {
    const auto& torus = message.toruses(i);
    if (torus.failure()) {
      continue;
    }
    QColor color =
        auto_color ? viz::ColorCategory20(static_cast<std::size_t>(i)) : flat;
    color.setAlphaF(alpha);
    const float radius = static_cast<float>(torus.large_radius());
    if (radius <= 0.f) {
      continue;
    }
    for (int s = 0; s < ring_segments; ++s) {
      const float a0 =
          static_cast<float>(2.0 * M_PI * s / ring_segments);
      const float a1 =
          static_cast<float>(2.0 * M_PI * (s + 1) / ring_segments);
      const QVector3D local0(radius * qCos(a0), radius * qSin(a0), 0.f);
      const QVector3D local1(radius * qCos(a1), radius * qSin(a1), 0.f);
      segments_.push_back(
          {viz::TransformPosePoint(tf, have_tf, torus.pose(), local0),
           viz::TransformPosePoint(tf, have_tf, torus.pose(), local1), color});
    }
  }
  if (context_->request_redraw) {
    context_->request_redraw();
  }
}

void TorusArrayDisplay::onDraw(rendering::SceneOverlay& scene) {
  std::vector<LineSegment3D> lines;
  lines.reserve(segments_.size());
  for (const LineVisual& seg : segments_) {
    lines.push_back({seg.start, seg.end, seg.color});
  }
  drawLineSegmentsOgreOrGl(context_, scene, name() + "/torus", lines);
}

}  // namespace display
}  // namespace autoviz
