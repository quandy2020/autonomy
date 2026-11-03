/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/plugins/displays/recognition/recognition_bounding_box_array_display.hpp"

#include <utility>

#include <QMatrix4x4>

#include "autoviz/plugins/displays/common/color_category.hpp"
#include "autoviz/plugins/displays/common/display_tf_utils.hpp"

namespace autoviz {
namespace display {

RecognitionBoundingBoxArrayDisplay::RecognitionBoundingBoxArrayDisplay(
    std::string channel)
    : ChannelDisplay<automsgs::msgs::visualization_msgs::BoundingBoxArray>(
          "RecognitionBoundingBoxArray", std::move(channel),
          "automsgs.msgs.visualization_msgs.BoundingBoxArray") {
  setProperties({});
}

std::vector<common::DisplayPropertySpec>
RecognitionBoundingBoxArrayDisplay::propertySpecs() const {
  return {{"coloring", "Coloring", "Auto", {"Auto", "Flat", "Label"}},
          {"color", "Color", "255;85;0", {}, common::DisplayPropertyKind::kColor},
          {"alpha", "Alpha", "0.8"},
          {"style", "Style", "Wireframe", {"Wireframe", "Solid"}}};
}

void RecognitionBoundingBoxArrayDisplay::clearReceivedData() {
  boxes_.clear();
}

void RecognitionBoundingBoxArrayDisplay::processMessage(
    const automsgs::msgs::visualization_msgs::BoundingBoxArray& message) {
  boxes_.clear();
  if (context_ == nullptr) {
    return;
  }
  automsgs::msgs::geometry_msgs::TransformStamped tf;
  const bool have_tf = viz::LookupFixedFrameTransform(
      context_, message.header().frame_id(), &tf);

  const std::string coloring = propertyValue("coloring", "Auto");
  QColor flat = common::ParseColorProperty(propertyValue("color", "255;85;0"),
                                           QColor(255, 85, 0));
  const float alpha =
      common::ParseFloatProperty(propertyValue("alpha", "0.8"), 0.8f);
  flat.setAlphaF(alpha);

  boxes_.reserve(static_cast<std::size_t>(message.boxes_size()));
  for (int i = 0; i < message.boxes_size(); ++i) {
    const auto& box = message.boxes(i);
    BoxVisual visual;
    QVector3D local(static_cast<float>(box.pose().position().x()),
                    static_cast<float>(box.pose().position().y()),
                    static_cast<float>(box.pose().position().z()));
    visual.center = viz::TransformLocalPoint(tf, have_tf, local);
    visual.size =
        QVector3D(static_cast<float>(box.dimensions().x()),
                  static_cast<float>(box.dimensions().y()),
                  static_cast<float>(box.dimensions().z()));
    visual.orientation = viz::QuatFromProto(box.pose().orientation());
    if (coloring == "Label") {
      visual.color = viz::ColorCategory20(box.label());
    } else if (coloring == "Auto") {
      visual.color = viz::ColorCategory20(static_cast<std::size_t>(i));
    } else {
      visual.color = flat;
    }
    visual.color.setAlphaF(alpha);
    boxes_.push_back(visual);
  }
  if (context_->request_redraw) {
    context_->request_redraw();
  }
}

void RecognitionBoundingBoxArrayDisplay::onDraw(
    rendering::SceneOverlay& scene) {
  const bool solid = propertyValue("style", "Wireframe") == "Solid";
  for (const BoxVisual& box : boxes_) {
    QMatrix4x4 transform;
    transform.translate(box.center);
    transform.rotate(box.orientation);
    const QVector3D half = box.size * 0.5f;
    if (solid) {
      scene.addBoxSolid(QVector3D(0.f, 0.f, 0.f), half, transform, box.color);
    } else {
      scene.addBoxWireframe(QVector3D(0.f, 0.f, 0.f), half, transform,
                            box.color);
    }
  }
}

}  // namespace display
}  // namespace autoviz
