/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/plugins/displays/recognition/segment_array_display.hpp"

#include <utility>

#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include "autoviz/commsgs/time_utils.hpp"
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include "autoviz/plugins/displays/common/ogre_overlay_draw.hpp"
#include "autoviz/plugins/displays/common/transform_utils.hpp"
#include "autoviz/plugins/displays/common/color_category.hpp"

namespace autoviz {
namespace display {

SegmentArrayDisplay::SegmentArrayDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::visualization_msgs::SegmentArray>(
          "SegmentArray", std::move(channel),
          "automsgs.msgs.visualization_msgs.SegmentArray") {
  setProperties({});
}

std::vector<common::DisplayPropertySpec>
SegmentArrayDisplay::propertySpecs() const {
  return {{"coloring", "Coloring", "Auto", {"Auto", "Flat"}},
          {"color", "Color", "25;255;240", {},
           common::DisplayPropertyKind::kColor},
          {"alpha", "Alpha", "1.0"},
          {"line_width", "Line Width", "0.02"}};
}

void SegmentArrayDisplay::clearReceivedData() { segments_.clear(); }

void SegmentArrayDisplay::processMessage(
    const automsgs::msgs::visualization_msgs::SegmentArray& message) {
  segments_.clear();
  if (context_ == nullptr) {
    return;
  }
  const auto zero_time = autoviz::commsgs::ZeroTime();
  const std::string frame = message.header().frame_id().empty()
                                ? context_->fixed_frame
                                : message.header().frame_id();
  automsgs::msgs::geometry_msgs::TransformStamped tf;
  bool have_tf = false;
  try {
    tf = context_->tf_buffer->lookupTransform(context_->fixed_frame, frame,
                                              zero_time);
    have_tf = true;
  } catch (...) {
  }

  const bool auto_color = propertyValue("coloring", "Auto") == "Auto";
  QColor flat = common::ParseColorProperty(
      propertyValue("color", "25;255;240"), QColor(25, 255, 240));
  const float alpha =
      common::ParseFloatProperty(propertyValue("alpha", "1.0"), 1.f);
  flat.setAlphaF(alpha);

  segments_.reserve(static_cast<std::size_t>(message.segments_size()));
  for (int i = 0; i < message.segments_size(); ++i) {
    const auto& seg = message.segments(i);
    SegmentVisual visual;
    QVector3D start(static_cast<float>(seg.start_point().x()),
                    static_cast<float>(seg.start_point().y()),
                    static_cast<float>(seg.start_point().z()));
    QVector3D end(static_cast<float>(seg.end_point().x()),
                  static_cast<float>(seg.end_point().y()),
                  static_cast<float>(seg.end_point().z()));
    visual.start = have_tf ? transformPoint(tf, start) : start;
    visual.end = have_tf ? transformPoint(tf, end) : end;
    visual.color =
        auto_color ? viz::ColorCategory20(static_cast<std::size_t>(i)) : flat;
    visual.color.setAlphaF(alpha);
    segments_.push_back(visual);
  }
  if (context_->request_redraw) {
    context_->request_redraw();
  }
}

void SegmentArrayDisplay::onDraw(rendering::SceneOverlay& scene) {
  std::vector<LineSegment3D> lines;
  lines.reserve(segments_.size());
  for (const SegmentVisual& seg : segments_) {
    lines.push_back({seg.start, seg.end, seg.color});
  }
  drawLineSegmentsOgreOrGl(context_, scene, name() + "/seg", lines);
}

}  // namespace display
}  // namespace autoviz
