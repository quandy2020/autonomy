/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/plugins/displays/footstep/footstep_display.hpp"

#include <utility>

#include <QMatrix4x4>

#include "autoviz/plugins/displays/common/display_tf_utils.hpp"

namespace autoviz {
namespace display {
namespace {

QColor ColorForLeg(automsgs::msgs::visualization_msgs::Footstep::Leg leg,
                   float alpha) {
  QColor color = leg == automsgs::msgs::visualization_msgs::Footstep::LEFT
                       ? QColor(60, 120, 220)
                       : QColor(220, 60, 60);
  color.setAlphaF(alpha);
  return color;
}

}  // namespace

FootstepDisplay::FootstepDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::visualization_msgs::FootstepArray>(
          "Footstep", std::move(channel),
          "automsgs.msgs.visualization_msgs.FootstepArray") {
  setProperties({});
}

std::vector<common::DisplayPropertySpec> FootstepDisplay::propertySpecs()
    const {
  return {{"alpha", "Alpha", "0.7"},
          {"style", "Style", "Wireframe", {"Wireframe", "Solid"}},
          {"default_length", "Default Length", "0.25"},
          {"default_width", "Default Width", "0.12"},
          {"default_height", "Default Height", "0.02"}};
}

void FootstepDisplay::clearReceivedData() { footsteps_.clear(); }

void FootstepDisplay::processMessage(
    const automsgs::msgs::visualization_msgs::FootstepArray& message) {
  footsteps_.clear();
  if (context_ == nullptr) {
    return;
  }
  automsgs::msgs::geometry_msgs::TransformStamped tf;
  const bool have_tf = viz::LookupFixedFrameTransform(
      context_, message.header().frame_id(), &tf);

  const float alpha =
      common::ParseFloatProperty(propertyValue("alpha", "0.7"), 0.7f);
  const float default_length =
      common::ParseFloatProperty(propertyValue("default_length", "0.25"), 0.25f);
  const float default_width =
      common::ParseFloatProperty(propertyValue("default_width", "0.12"), 0.12f);
  const float default_height =
      common::ParseFloatProperty(propertyValue("default_height", "0.02"), 0.02f);

  footsteps_.reserve(static_cast<std::size_t>(message.footsteps_size()));
  for (int i = 0; i < message.footsteps_size(); ++i) {
    const auto& step = message.footsteps(i);
    BoxVisual visual;
    QVector3D local(static_cast<float>(step.pose().position().x()),
                    static_cast<float>(step.pose().position().y()),
                    static_cast<float>(step.pose().position().z()));
    visual.center = viz::TransformLocalPoint(tf, have_tf, local);
    visual.orientation = viz::QuatFromProto(step.pose().orientation());
    const float dx = step.dimensions().x() > 0.f
                         ? static_cast<float>(step.dimensions().x())
                         : default_length;
    const float dy = step.dimensions().y() > 0.f
                         ? static_cast<float>(step.dimensions().y())
                         : default_width;
    const float dz = step.dimensions().z() > 0.f
                         ? static_cast<float>(step.dimensions().z())
                         : default_height;
    visual.size = QVector3D(dx, dy, dz);
    visual.color = ColorForLeg(step.leg(), alpha);
    footsteps_.push_back(visual);
  }
  if (context_->request_redraw) {
    context_->request_redraw();
  }
}

void FootstepDisplay::onDraw(rendering::SceneOverlay& scene) {
  const bool solid = propertyValue("style", "Wireframe") == "Solid";
  for (const BoxVisual& step : footsteps_) {
    QMatrix4x4 transform;
    transform.translate(step.center);
    transform.rotate(step.orientation);
    const QVector3D half = step.size * 0.5f;
    if (solid) {
      scene.addBoxSolid(QVector3D(0.f, 0.f, 0.f), half, transform, step.color);
    } else {
      scene.addBoxWireframe(QVector3D(0.f, 0.f, 0.f), half, transform,
                            step.color);
    }
  }
}

}  // namespace display
}  // namespace autoviz
