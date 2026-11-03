/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/plugins/displays/recognition/bounding_box_array_display.hpp"

#include <utility>

#include <QMatrix4x4>

#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include "autoviz/commsgs/time_utils.hpp"
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include "autoviz/plugins/displays/common/transform_utils.hpp"
#include "autoviz/plugins/displays/common/color_category.hpp"

namespace autoviz {
namespace display {
namespace {

QQuaternion QuatFromProto(
    const automsgs::msgs::geometry_msgs::Quaternion& q) {
  return QQuaternion(static_cast<float>(q.w()), static_cast<float>(q.x()),
                     static_cast<float>(q.y()), static_cast<float>(q.z()));
}

}  // namespace

BoundingBox3DArrayDisplay::BoundingBox3DArrayDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::vision_msgs::BoundingBox3DArray>(
          "BoundingBox3DArray", std::move(channel),
          "automsgs.msgs.vision_msgs.BoundingBox3DArray") {
  setProperties({});
}

std::vector<common::DisplayPropertySpec>
BoundingBox3DArrayDisplay::propertySpecs() const {
  return {{"coloring", "Coloring", "Auto", {"Auto", "Flat"}},
          {"color", "Color", "255;85;0", {}, common::DisplayPropertyKind::kColor},
          {"alpha", "Alpha", "0.8"},
          {"style", "Style", "Wireframe", {"Wireframe", "Solid"}}};
}

void BoundingBox3DArrayDisplay::clearReceivedData() { boxes_.clear(); }

void BoundingBox3DArrayDisplay::processMessage(
    const automsgs::msgs::vision_msgs::BoundingBox3DArray& message) {
  boxes_.clear();
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
  QColor flat = common::ParseColorProperty(propertyValue("color", "255;85;0"),
                                           QColor(255, 85, 0));
  const float alpha =
      common::ParseFloatProperty(propertyValue("alpha", "0.8"), 0.8f);
  flat.setAlphaF(alpha);

  boxes_.reserve(static_cast<std::size_t>(message.boxes_size()));
  for (int i = 0; i < message.boxes_size(); ++i) {
    const auto& box = message.boxes(i);
    BoxVisual visual;
    QVector3D local(static_cast<float>(box.center().position().x()),
                    static_cast<float>(box.center().position().y()),
                    static_cast<float>(box.center().position().z()));
    visual.center = have_tf ? transformPoint(tf, local) : local;
    visual.size =
        QVector3D(static_cast<float>(box.size().x()),
                  static_cast<float>(box.size().y()),
                  static_cast<float>(box.size().z()));
    visual.orientation = QuatFromProto(box.center().orientation());
    visual.color = auto_color ? viz::ColorCategory20(static_cast<std::size_t>(i))
                              : flat;
    visual.color.setAlphaF(alpha);
    boxes_.push_back(visual);
  }
  if (context_->request_redraw) {
    context_->request_redraw();
  }
}

void BoundingBox3DArrayDisplay::onDraw(rendering::SceneOverlay& scene) {
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
