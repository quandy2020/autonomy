/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/strata_robot_display.hpp"

#include <QtMath>

#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/commsgs/time_utils.hpp"
#include "autoviz/display/ogre_colored_points_draw.hpp"
#include "autoviz/display/ogre_label_draw.hpp"
#include "autoviz/display/ogre_overlay_draw.hpp"
#include "autoviz/display/transform_utils.hpp"

namespace autoviz {
namespace display {
namespace {

QColor StatusColor(const std::string& status) {
  if (status == "running") {
    return QColor(0, 102, 255);
  }
  if (status == "charging") {
    return QColor(247, 168, 0);
  }
  if (status == "error") {
    return QColor(255, 59, 48);
  }
  if (status == "returning") {
    return QColor(6, 182, 212);
  }
  return QColor(100, 116, 139);
}

}  // namespace

StrataRobotDisplay::StrataRobotDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::strata_msgs::RobotMarkerArray>(
          "StrataRobot", std::move(channel),
          "automsgs.msgs.strata_msgs.RobotMarkerArray") {
  setProperties({});
}

std::vector<common::DisplayPropertySpec> StrataRobotDisplay::propertySpecs()
    const {
  using common::DisplayPropertyKind;
  return {{"color", "Color", "0;102;255", {}, DisplayPropertyKind::kColor},
          {"axis_length", "Axis Length", "0.4"},
          {"use_status_color", "Use Status Color", "true"},
          {"show_labels", "Show Labels", "true"},
          {"label_height", "Label Height", "0.12"}};
}

void StrataRobotDisplay::onPropertyChanged(const std::string& /*key*/) {
  if (context_ != nullptr && context_->request_redraw) {
    context_->request_redraw();
  }
}

void StrataRobotDisplay::processMessage(
    const automsgs::msgs::strata_msgs::RobotMarkerArray& message) {
  robots_.clear();
  if (context_ == nullptr) {
    return;
  }

  const auto zero_time = autoviz::commsgs::ZeroTime();
  const std::string frame = message.header().frame_id().empty()
                                ? context_->fixed_frame
                                : message.header().frame_id();
  const bool use_status_color =
      common::ParseBoolProperty(propertyValue("use_status_color", "true"), true);

  for (const auto& robot : message.robots()) {
    QVector3D local(static_cast<float>(robot.lng_lat().x()),
                    static_cast<float>(robot.lng_lat().y()),
                    static_cast<float>(robot.lng_lat().z()));
    if (frame != context_->fixed_frame) {
      try {
        const auto tf = context_->tf_buffer->lookupTransform(
            context_->fixed_frame, frame, zero_time);
        local = transformPoint(tf, local);
      } catch (...) {
        continue;
      }
    }
    StoredRobot stored;
    stored.position = local;
    stored.yaw = static_cast<float>(robot.rotation_deg() * M_PI / 180.0);
    stored.status = robot.status();
    stored.battery = robot.battery();
    if (!robot.name().empty()) {
      stored.label = QString::fromStdString(robot.name());
    } else {
      stored.label = QString::fromStdString(robot.id());
    }
    robots_.push_back(std::move(stored));
  }

  if (context_->request_redraw) {
    context_->request_redraw();
  }
}

void StrataRobotDisplay::clearReceivedData() {
  robots_.clear();
}

void StrataRobotDisplay::onDraw(rendering::SceneOverlay& scene) {
  const float len =
      common::ParseFloatProperty(propertyValue("axis_length", "0.4"), 0.4f);
  const bool show_labels =
      common::ParseBoolProperty(propertyValue("show_labels", "true"), true);
  const float label_height =
      common::ParseFloatProperty(propertyValue("label_height", "0.12"), 0.12f);
  const bool use_status_color =
      common::ParseBoolProperty(propertyValue("use_status_color", "true"), true);
  const QColor custom_color = common::ParseColorProperty(
      propertyValue("color", "0;102;255"), QColor(0, 102, 255));
  std::vector<TextLabelInstance> labels;
  labels.reserve(robots_.size() * 2);

  for (size_t i = 0; i < robots_.size(); ++i) {
    const auto& robot = robots_[i];
    const QColor robot_color =
        use_status_color ? StatusColor(robot.status) : custom_color;
    const QVector3D heading(len * qCos(robot.yaw), len * qSin(robot.yaw), 0.f);
    const std::string suffix = "/robot/" + std::to_string(i);
    drawArrowOgreOrGl(context_, scene, name() + suffix + "/heading", robot.position,
                      robot.position + heading, robot_color);
    drawColoredPointsOgreOrGl(context_, scene, name() + suffix + "/point", typeId(),
                              4.f, rendering::PointCloudStyle::kSquares,
                              {{robot.position, robot_color}}, false);

    if (!show_labels) {
      continue;
    }
    const QVector3D label_pos = robot.position + QVector3D(0.f, 0.f, 0.08f);
    TextLabelInstance name_label;
    name_label.text = robot.label.toStdString();
    name_label.position = label_pos;
    name_label.color = robot_color;
    name_label.char_height = label_height;
    labels.push_back(std::move(name_label));

    if (robot.battery >= 0.f) {
      TextLabelInstance battery_label;
      const QString status_text =
          robot.status.empty() ? QString() : QString::fromStdString(robot.status);
      battery_label.text =
          QString("%1% %2")
              .arg(static_cast<int>(robot.battery))
              .arg(status_text)
              .trimmed()
              .toStdString();
      battery_label.position = label_pos + QVector3D(0.f, 0.f, -label_height * 1.2f);
      battery_label.color = robot.battery < 20.f ? QColor(255, 59, 48) : QColor(100, 116, 139);
      battery_label.char_height = label_height * 0.85f;
      labels.push_back(std::move(battery_label));
    }
  }

  if (!labels.empty()) {
    drawLabelsOgreOrGl(context_, scene, name() + "/labels", labels);
  }
}

}  // namespace display
}  // namespace autoviz
