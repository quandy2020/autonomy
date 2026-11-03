/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/plugins/displays/recognition/human_skeleton_array_display.hpp"

#include <utility>

#include "autoviz/plugins/displays/common/ogre_overlay_draw.hpp"
#include "autoviz/plugins/displays/common/color_category.hpp"
#include "autoviz/plugins/displays/common/display_tf_utils.hpp"

namespace autoviz {
namespace display {

HumanSkeletonArrayDisplay::HumanSkeletonArrayDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::visualization_msgs::HumanSkeletonArray>(
          "HumanSkeletonArray", std::move(channel),
          "automsgs.msgs.visualization_msgs.HumanSkeletonArray") {
  setProperties({});
}

std::vector<common::DisplayPropertySpec>
HumanSkeletonArrayDisplay::propertySpecs() const {
  return {{"coloring", "Coloring", "Auto", {"Auto", "Flat"}},
          {"color", "Color", "214;39;40", {}, common::DisplayPropertyKind::kColor},
          {"alpha", "Alpha", "1.0"}};
}

void HumanSkeletonArrayDisplay::clearReceivedData() { segments_.clear(); }

void HumanSkeletonArrayDisplay::processMessage(
    const automsgs::msgs::visualization_msgs::HumanSkeletonArray& message) {
  segments_.clear();
  if (context_ == nullptr) {
    return;
  }
  automsgs::msgs::geometry_msgs::TransformStamped array_tf;
  const bool have_array_tf = viz::LookupFixedFrameTransform(
      context_, message.header().frame_id(), &array_tf);

  const bool auto_color = propertyValue("coloring", "Auto") == "Auto";
  QColor flat = common::ParseColorProperty(propertyValue("color", "214;39;40"),
                                           QColor(214, 39, 40));
  const float alpha =
      common::ParseFloatProperty(propertyValue("alpha", "1.0"), 1.f);
  flat.setAlphaF(alpha);

  for (int h = 0; h < message.skeletons_size(); ++h) {
    const auto& skeleton = message.skeletons(h);
    automsgs::msgs::geometry_msgs::TransformStamped tf = array_tf;
    bool have_tf = have_array_tf;
    if (!skeleton.header().frame_id().empty()) {
      have_tf = viz::LookupFixedFrameTransform(
          context_, skeleton.header().frame_id(), &tf);
    }
    QColor color =
        auto_color ? viz::ColorCategory20(static_cast<std::size_t>(h)) : flat;
    color.setAlphaF(alpha);
    for (int b = 0; b < skeleton.bones_size(); ++b) {
      const auto& bone = skeleton.bones(b);
      QVector3D start(static_cast<float>(bone.start_point().x()),
                      static_cast<float>(bone.start_point().y()),
                      static_cast<float>(bone.start_point().z()));
      QVector3D end(static_cast<float>(bone.end_point().x()),
                    static_cast<float>(bone.end_point().y()),
                    static_cast<float>(bone.end_point().z()));
      segments_.push_back({viz::TransformLocalPoint(tf, have_tf, start),
                           viz::TransformLocalPoint(tf, have_tf, end), color});
    }
  }
  if (context_->request_redraw) {
    context_->request_redraw();
  }
}

void HumanSkeletonArrayDisplay::onDraw(rendering::SceneOverlay& scene) {
  std::vector<LineSegment3D> lines;
  lines.reserve(segments_.size());
  for (const LineVisual& seg : segments_) {
    lines.push_back({seg.start, seg.end, seg.color});
  }
  drawLineSegmentsOgreOrGl(context_, scene, name() + "/skeleton", lines);
}

}  // namespace display
}  // namespace autoviz
