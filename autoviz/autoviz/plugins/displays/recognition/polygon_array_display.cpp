/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/plugins/displays/recognition/polygon_array_display.hpp"

#include <utility>

#include "autoviz/plugins/displays/common/ogre_overlay_draw.hpp"
#include "autoviz/plugins/displays/common/color_category.hpp"
#include "autoviz/plugins/displays/common/display_tf_utils.hpp"

namespace autoviz {
namespace display {

PolygonArrayDisplay::PolygonArrayDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::visualization_msgs::PolygonArray>(
          "PolygonArray", std::move(channel),
          "automsgs.msgs.visualization_msgs.PolygonArray") {
  setProperties({});
}

std::vector<common::DisplayPropertySpec>
PolygonArrayDisplay::propertySpecs() const {
  return {{"coloring", "Coloring", "Auto", {"Auto", "Flat", "Label"}},
          {"color", "Color", "44;160;44", {}, common::DisplayPropertyKind::kColor},
          {"alpha", "Alpha", "1.0"}};
}

void PolygonArrayDisplay::clearReceivedData() { segments_.clear(); }

void PolygonArrayDisplay::processMessage(
    const automsgs::msgs::visualization_msgs::PolygonArray& message) {
  segments_.clear();
  if (context_ == nullptr) {
    return;
  }
  automsgs::msgs::geometry_msgs::TransformStamped array_tf;
  const bool have_array_tf = viz::LookupFixedFrameTransform(
      context_, message.header().frame_id(), &array_tf);

  const std::string coloring = propertyValue("coloring", "Auto");
  QColor flat = common::ParseColorProperty(propertyValue("color", "44;160;44"),
                                           QColor(44, 160, 44));
  const float alpha =
      common::ParseFloatProperty(propertyValue("alpha", "1.0"), 1.f);
  flat.setAlphaF(alpha);

  for (int i = 0; i < message.polygons_size(); ++i) {
    const auto& poly_stamped = message.polygons(i);
    automsgs::msgs::geometry_msgs::TransformStamped tf = array_tf;
    bool have_tf = have_array_tf;
    if (!poly_stamped.header().frame_id().empty()) {
      have_tf = viz::LookupFixedFrameTransform(
          context_, poly_stamped.header().frame_id(), &tf);
    }
    QColor color;
    if (coloring == "Label" && i < message.labels_size()) {
      color = viz::ColorCategory20(message.labels(i));
    } else if (coloring == "Auto") {
      color = viz::ColorCategory20(static_cast<std::size_t>(i));
    } else {
      color = flat;
    }
    color.setAlphaF(alpha);

    const auto& polygon = poly_stamped.polygon();
    if (polygon.points_size() < 2) {
      continue;
    }
    std::vector<QVector3D> ring;
    ring.reserve(static_cast<std::size_t>(polygon.points_size()));
    for (int p = 0; p < polygon.points_size(); ++p) {
      const auto& pt = polygon.points(p);
      ring.push_back(viz::TransformLocalPoint(
          tf, have_tf,
          QVector3D(static_cast<float>(pt.x()), static_cast<float>(pt.y()),
                    static_cast<float>(pt.z()))));
    }
    for (std::size_t p = 0; p < ring.size(); ++p) {
      const std::size_t next = (p + 1) % ring.size();
      segments_.push_back({ring[p], ring[next], color});
    }
  }
  if (context_->request_redraw) {
    context_->request_redraw();
  }
}

void PolygonArrayDisplay::onDraw(rendering::SceneOverlay& scene) {
  std::vector<LineSegment3D> lines;
  lines.reserve(segments_.size());
  for (const LineVisual& seg : segments_) {
    lines.push_back({seg.start, seg.end, seg.color});
  }
  drawLineSegmentsOgreOrGl(context_, scene, name() + "/poly", lines);
}

}  // namespace display
}  // namespace autoviz
