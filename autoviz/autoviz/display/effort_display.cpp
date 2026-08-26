/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/effort_display.hpp"

#include <cmath>

#include <QMatrix4x4>
#include <QtMath>

#include "autolink/message/raw_message.hpp"
#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include "autoviz/commsgs/time_utils.hpp"
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/transform_utils.hpp"

namespace autoviz {
namespace display {
namespace {

QMatrix4x4 JointOriginMatrix(const UrdfJoint& joint) {
  QMatrix4x4 matrix;
  matrix.setToIdentity();
  matrix.translate(joint.origin);
  matrix.rotate(joint.rotation);
  return matrix;
}

QMatrix4x4 JointMotionMatrix(const UrdfJoint& joint, double position) {
  QMatrix4x4 matrix;
  matrix.setToIdentity();
  if (joint.type == UrdfJointType::kPrismatic) {
    matrix.translate(joint.axis * static_cast<float>(position));
  } else if (joint.type == UrdfJointType::kRevolute ||
             joint.type == UrdfJointType::kContinuous) {
    matrix.rotate(static_cast<float>(qRadiansToDegrees(position)), joint.axis);
  }
  return matrix;
}

}  // namespace

EffortDisplay::EffortDisplay(std::string joint_channel)
    : joint_channel_(std::move(joint_channel)) {}

void EffortDisplay::setChannel(const std::string& channel) {
  if (joint_channel_ == channel) {
    return;
  }
  const bool active = enabled();
  if (active) {
    onDisable();
  }
  joint_channel_ = channel;
  if (active) {
    onEnable();
  }
}

std::vector<common::DisplayPropertySpec> EffortDisplay::propertySpecs() const {
  return {{"urdf_path", "URDF Path", ""},
          {"description_channel", "Description Channel", "/robot_description"},
          {"root_link", "Root Link", ""},
          {"effort_scale", "Effort Scale", "0.02"},
          {"min_length", "Min Arrow Length", "0.04"},
          {"positive_color", "Positive Color", "255;80;80"},
          {"negative_color", "Negative Color", "80;160;255"},
          {"effort_threshold", "Effort Threshold", "0.01"}};
}

void EffortDisplay::onPropertyChanged(const std::string& key) {
  if (key == "urdf_path" || key == "description_channel") {
    reloadUrdf();
  }
}

void EffortDisplay::reloadUrdf() {
  const std::string path = propertyValue("urdf_path", "");
  if (!path.empty()) {
    model_.loadFromFile(path);
  }
}

void EffortDisplay::onEnable() {
  if (context_ == nullptr || context_->autolink == nullptr ||
      context_->autolink->node() == nullptr) {
    return;
  }
  reloadUrdf();

  const std::string description_channel =
      propertyValue("description_channel", "/robot_description");
  description_reader_.reset();
  if (!description_channel.empty()) {
    description_reader_ =
        context_->autolink->node()->CreateReader<autolink::message::RawMessage>(
            description_channel,
            [this](const std::shared_ptr<autolink::message::RawMessage>& msg) {
              if (msg == nullptr) {
                return;
              }
              std::string urdf_text;
              if (proto_wire::UnwrapStdStringPayload(msg->message, &urdf_text) &&
                  model_.loadFromString(urdf_text) && context_ != nullptr &&
                  context_->request_redraw) {
                context_->request_redraw();
              }
            });
  }

  joint_reader_ =
      context_->autolink->node()->CreateReader<autolink::message::RawMessage>(
          joint_channel_,
          [this](const std::shared_ptr<autolink::message::RawMessage>& msg) {
            if (msg != nullptr) {
              joint_queue_.push(msg->message);
            }
          });
}

void EffortDisplay::onDisable() {
  joint_reader_.reset();
  description_reader_.reset();
}

void EffortDisplay::reset() {
  Display::reset();
  joint_queue_.clear();
  joint_positions_.clear();
  joint_efforts_.clear();
  if (context_ != nullptr && context_->request_redraw) {
    context_->request_redraw();
  }
}

void EffortDisplay::onUpdate() {
  while (auto payload = joint_queue_.pop()) {
    proto_wire::ParsedJointState message;
    if (proto_wire::ParseJointStatePayload(*payload, &message)) {
      processJointState(message);
    }
  }
}

void EffortDisplay::processJointState(
    const proto_wire::ParsedJointState& message) {
  const size_t name_count = message.names.size();
  const size_t position_count = message.positions.size();
  for (size_t i = 0; i < name_count && i < position_count; ++i) {
    joint_positions_[message.names[i]] = message.positions[i];
  }

  joint_efforts_.clear();
  const size_t effort_count = message.efforts.size();
  for (size_t i = 0; i < name_count && i < effort_count; ++i) {
    joint_efforts_[message.names[i]] = message.efforts[i];
  }

  if (context_ != nullptr && context_->request_redraw) {
    context_->request_redraw();
  }
}

void EffortDisplay::drawEffortArrow(rendering::SceneOverlay& scene,
                                    const QVector3D& origin,
                                    const QVector3D& axis, double effort,
                                    const QColor& positive_color,
                                    const QColor& negative_color,
                                    float min_length, float scale) const {
  const float magnitude = static_cast<float>(std::abs(effort));
  if (magnitude <= 1e-6f) {
    return;
  }
  const QColor color = effort >= 0.0 ? positive_color : negative_color;
  const float length = std::max(min_length, magnitude * scale);
  const QVector3D direction =
      effort >= 0.0 ? axis.normalized() : -axis.normalized();
  const QVector3D end = origin + direction * length;
  scene.addLine(origin, end, color);
  QVector3D side = QVector3D::crossProduct(direction, QVector3D(0.f, 0.f, 1.f));
  if (side.lengthSquared() < 1e-4f) {
    side = QVector3D::crossProduct(direction, QVector3D(0.f, 1.f, 0.f));
  }
  side.normalize();
  const float head = std::min(0.12f, length * 0.25f);
  scene.addLine(end, end - direction * head + side * head * 0.35f, color);
  scene.addLine(end, end - direction * head - side * head * 0.35f, color);
  scene.addPoint(origin, color);

  const int segments = 12;
  QVector3D prev = origin + side * (length * 0.35f);
  for (int i = 1; i <= segments; ++i) {
    const float angle = static_cast<float>(i) * 6.2831853f /
                        static_cast<float>(segments) *
                        static_cast<float>(effort >= 0.0 ? 1.0 : -1.0);
    const QVector3D radial =
        side * std::cos(angle) * (length * 0.35f) +
        QVector3D::crossProduct(direction, side).normalized() *
            std::sin(angle) * (length * 0.35f);
    const QVector3D next = origin + radial;
    scene.addLine(prev, next, color);
    prev = next;
  }
}

void EffortDisplay::onDraw(rendering::SceneOverlay& scene) {
  if (context_ == nullptr || model_.empty() || joint_efforts_.empty()) {
    return;
  }

  const float effort_scale =
      common::ParseFloatProperty(propertyValue("effort_scale", "0.02"), 0.02f);
  const float min_length = common::ParseFloatProperty(
      propertyValue("min_length", "0.04"), 0.04f);
  const float effort_threshold = common::ParseFloatProperty(
      propertyValue("effort_threshold", "0.01"), 0.01f);
  const QColor positive_color =
      common::ParseColorProperty(propertyValue("positive_color", "255;80;80"));
  const QColor negative_color =
      common::ParseColorProperty(propertyValue("negative_color", "80;160;255"));

  const auto link_transforms = model_.computeLinkTransforms(joint_positions_);

  QMatrix4x4 root_world;
  root_world.setToIdentity();
  const std::string root_link = propertyValue("root_link", model_.rootLink());
  if (!root_link.empty() && context_->tf_buffer != nullptr) {
    const auto zero_time = autoviz::commsgs::ZeroTime();
    try {
      const auto tf = context_->tf_buffer->lookupTransform(
          context_->fixed_frame, root_link, zero_time);
      root_world = transformToMatrix(tf);
    } catch (...) {
    }
  }

  for (const auto& joint : model_.joints()) {
    const auto effort_it = joint_efforts_.find(joint.name);
    if (effort_it == joint_efforts_.end() ||
        std::abs(effort_it->second) < effort_threshold) {
      continue;
    }
    const auto parent_it = link_transforms.find(joint.parent);
    if (parent_it == link_transforms.end()) {
      continue;
    }

    double position = 0.0;
    const auto position_it = joint_positions_.find(joint.name);
    if (position_it != joint_positions_.end()) {
      position = position_it->second;
    }

    const QMatrix4x4 joint_frame =
        root_world * parent_it->second * JointOriginMatrix(joint) *
        JointMotionMatrix(joint, position * 0.5);
    const QVector3D origin = joint_frame.map(QVector3D(0.f, 0.f, 0.f));
    const QVector3D axis_tip = joint_frame.map(joint.axis);
    QVector3D axis = axis_tip - origin;
    if (axis.lengthSquared() < 1e-6f) {
      continue;
    }
    axis.normalize();
    drawEffortArrow(scene, origin, axis, effort_it->second, positive_color,
                    negative_color, min_length, effort_scale);
  }
}

}  // namespace display
}  // namespace autoviz
