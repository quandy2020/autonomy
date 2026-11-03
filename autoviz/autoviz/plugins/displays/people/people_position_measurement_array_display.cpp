/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/plugins/displays/people/people_position_measurement_array_display.hpp"

#include <algorithm>
#include <utility>

#include <QMatrix4x4>

#include "autoviz/plugins/displays/common/color_category.hpp"
#include "autoviz/plugins/displays/common/display_tf_utils.hpp"

namespace autoviz {
namespace display {

PeoplePositionMeasurementArrayDisplay::
    PeoplePositionMeasurementArrayDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::people_msgs::PositionMeasurementArray>(
          "PeoplePositionMeasurementArray", std::move(channel),
          "automsgs.msgs.people_msgs.PositionMeasurementArray") {
  setProperties({});
}

std::vector<common::DisplayPropertySpec>
PeoplePositionMeasurementArrayDisplay::propertySpecs() const {
  return {{"coloring", "Coloring", "Auto", {"Auto", "Flat"}},
          {"color", "Color", "148;103;189", {},
           common::DisplayPropertyKind::kColor},
          {"alpha", "Alpha", "0.85"},
          {"radius", "Radius", "0.25"},
          {"style", "Style", "Points", {"Points", "Spheres"}}};
}

void PeoplePositionMeasurementArrayDisplay::clearReceivedData() {
  people_.clear();
}

void PeoplePositionMeasurementArrayDisplay::processMessage(
    const automsgs::msgs::people_msgs::PositionMeasurementArray& message) {
  people_.clear();
  if (context_ == nullptr) {
    return;
  }
  automsgs::msgs::geometry_msgs::TransformStamped array_tf;
  const bool have_array_tf = viz::LookupFixedFrameTransform(
      context_, message.header().frame_id(), &array_tf);

  const bool auto_color = propertyValue("coloring", "Auto") == "Auto";
  QColor flat = common::ParseColorProperty(
      propertyValue("color", "148;103;189"), QColor(148, 103, 189));
  const float base_alpha =
      common::ParseFloatProperty(propertyValue("alpha", "0.85"), 0.85f);
  const float radius =
      common::ParseFloatProperty(propertyValue("radius", "0.25"), 0.25f);

  people_.reserve(static_cast<std::size_t>(message.people_size()));
  for (int i = 0; i < message.people_size(); ++i) {
    const auto& person = message.people(i);
    automsgs::msgs::geometry_msgs::TransformStamped tf = array_tf;
    bool have_tf = have_array_tf;
    if (!person.header().frame_id().empty()) {
      have_tf = viz::LookupFixedFrameTransform(
          context_, person.header().frame_id(), &tf);
    }
    QColor color =
        auto_color ? viz::ColorCategory20(static_cast<std::size_t>(i)) : flat;
    const float reliability = static_cast<float>(
        std::clamp(person.reliability(), 0.0, 1.0));
    color.setAlphaF(base_alpha * std::max(0.1f, reliability));
    PersonVisual visual;
    visual.position = viz::TransformLocalPoint(
        tf, have_tf,
        QVector3D(static_cast<float>(person.pos().x()),
                  static_cast<float>(person.pos().y()),
                  static_cast<float>(person.pos().z())));
    visual.color = color;
    visual.radius = radius;
    people_.push_back(visual);
  }
  if (context_->request_redraw) {
    context_->request_redraw();
  }
}

void PeoplePositionMeasurementArrayDisplay::onDraw(
    rendering::SceneOverlay& scene) {
  const bool spheres = propertyValue("style", "Points") == "Spheres";
  for (const PersonVisual& person : people_) {
    if (spheres) {
      QMatrix4x4 transform;
      transform.translate(person.position);
      const QVector3D half(person.radius, person.radius, person.radius);
      scene.addBoxSolid(QVector3D(0.f, 0.f, 0.f), half, transform,
                        person.color);
    } else {
      scene.addPoint(person.position, person.color);
    }
  }
}

}  // namespace display
}  // namespace autoviz
