/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/plugins/displays/recognition/bounding_box_display.hpp"

#include <utility>

#include <QMatrix4x4>

#include "autoviz/plugins/displays/common/color_category.hpp"
#include "autoviz/plugins/displays/common/display_tf_utils.hpp"

namespace autoviz {
namespace display {

BoundingBoxDisplay::BoundingBoxDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::visualization_msgs::BoundingBox>(
          "BoundingBox", std::move(channel),
          "automsgs.msgs.visualization_msgs.BoundingBox") {
  setProperties({});
}

std::vector<common::DisplayPropertySpec>
BoundingBoxDisplay::propertySpecs() const {
  return {{"coloring", "Coloring", "Auto", {"Auto", "Flat", "Label"}},
          {"color", "Color", "255;85;0", {}, common::DisplayPropertyKind::kColor},
          {"alpha", "Alpha", "0.8"},
          {"style", "Style", "Wireframe", {"Wireframe", "Solid"}}};
}

void BoundingBoxDisplay::clearReceivedData() { boxes_.clear(); }

void BoundingBoxDisplay::processMessage(
    const automsgs::msgs::visualization_msgs::BoundingBox& message) {
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

  BoxVisual visual;
  QVector3D local(static_cast<float>(message.pose().position().x()),
                  static_cast<float>(message.pose().position().y()),
                  static_cast<float>(message.pose().position().z()));
  visual.center = viz::TransformLocalPoint(tf, have_tf, local);
  visual.size =
      QVector3D(static_cast<float>(message.dimensions().x()),
                static_cast<float>(message.dimensions().y()),
                static_cast<float>(message.dimensions().z()));
  visual.orientation = viz::QuatFromProto(message.pose().orientation());
  if (coloring == "Label") {
    visual.color = viz::ColorCategory20(message.label());
  } else if (coloring == "Auto") {
    visual.color = viz::ColorCategory20(0);
  } else {
    visual.color = flat;
  }
  visual.color.setAlphaF(alpha);
  boxes_.push_back(visual);

  if (context_->request_redraw) {
    context_->request_redraw();
  }
}

void BoundingBoxDisplay::onDraw(rendering::SceneOverlay& scene) {
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
