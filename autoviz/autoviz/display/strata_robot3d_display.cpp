/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/strata_robot3d_display.hpp"

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

StrataRobot3DDisplay::StrataRobot3DDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::strata_msgs::Robot3DLayerArray>(
          "StrataRobot3D", std::move(channel),
          "automsgs.msgs.strata_msgs.Robot3DLayerArray") {
  setProperties({});
}

std::vector<common::DisplayPropertySpec> StrataRobot3DDisplay::propertySpecs()
    const {
  using common::DisplayPropertyKind;
  return {{"color", "Color", "0;102;255", {}, DisplayPropertyKind::kColor},
          {"axis_length", "Axis Length", "0.5"},
          {"use_status_color", "Use Status Color", "true"},
          {"show_labels", "Show Labels", "true"},
          {"label_height", "Label Height", "0.1"}};
}

void StrataRobot3DDisplay::onPropertyChanged(const std::string& /*key*/) {
  if (context_ != nullptr && context_->request_redraw) {
    context_->request_redraw();
  }
}

void StrataRobot3DDisplay::processMessage(
    const automsgs::msgs::strata_msgs::Robot3DLayerArray& message) {
  layers_.clear();
  if (context_ == nullptr) {
    return;
  }

  const auto zero_time = autoviz::commsgs::ZeroTime();
  const std::string frame = message.header().frame_id().empty()
                                ? context_->fixed_frame
                                : message.header().frame_id();

  for (const auto& layer : message.layers()) {
    if (!layer.visible()) {
      continue;
    }
    QVector3D local(static_cast<float>(layer.position().x()),
                    static_cast<float>(layer.position().y()),
                    static_cast<float>(layer.position().z()));
    local += QVector3D(static_cast<float>(layer.offset().x()),
                       static_cast<float>(layer.offset().y()),
                       static_cast<float>(layer.offset().z()));
    if (frame != context_->fixed_frame) {
      try {
        const auto tf = context_->tf_buffer->lookupTransform(
            context_->fixed_frame, frame, zero_time);
        local = transformPoint(tf, local);
      } catch (...) {
        continue;
      }
    }
    StoredLayer stored;
    stored.position = local;
    stored.yaw = static_cast<float>((layer.heading_deg() + layer.rotation_deg()) * M_PI / 180.0);
    stored.status = layer.status();
    stored.model_url = QString::fromStdString(layer.model_url());
    stored.scale = static_cast<float>(layer.scale() > 0. ? layer.scale() : 1.);
    if (!layer.robot_id().empty()) {
      stored.label = QString::fromStdString(layer.robot_id());
    } else {
      stored.label = QString::fromStdString(layer.id());
    }
    layers_.push_back(std::move(stored));
  }

  if (context_->request_redraw) {
    context_->request_redraw();
  }
}

void StrataRobot3DDisplay::clearReceivedData() { layers_.clear(); }

void StrataRobot3DDisplay::onDraw(rendering::SceneOverlay& scene) {
  const float axis_len =
      common::ParseFloatProperty(propertyValue("axis_length", "0.5"), 0.5f);
  const bool show_labels =
      common::ParseBoolProperty(propertyValue("show_labels", "true"), true);
  const float label_height =
      common::ParseFloatProperty(propertyValue("label_height", "0.1"), 0.1f);
  const bool use_status_color =
      common::ParseBoolProperty(propertyValue("use_status_color", "true"), true);
  const QColor custom_color = common::ParseColorProperty(
      propertyValue("color", "0;102;255"), QColor(0, 102, 255));

  std::vector<TextLabelInstance> labels;
  labels.reserve(layers_.size() * 2);

  for (size_t i = 0; i < layers_.size(); ++i) {
    const auto& layer = layers_[i];
    const QColor layer_color =
        use_status_color ? StatusColor(layer.status) : custom_color;
    const float len = axis_len * layer.scale;
    const QVector3D heading(len * qCos(layer.yaw), len * qSin(layer.yaw), 0.f);
    const std::string suffix = "/robot3d/" + std::to_string(i);
    drawArrowOgreOrGl(context_, scene, name() + suffix + "/heading", layer.position,
                      layer.position + heading, layer_color);
    drawColoredPointsOgreOrGl(context_, scene, name() + suffix + "/body", typeId(), 6.f,
                              rendering::PointCloudStyle::kSquares,
                              {{layer.position, layer_color}}, false);

    if (!show_labels) {
      continue;
    }
    TextLabelInstance name_label;
    name_label.text = layer.label.toStdString();
    name_label.position = layer.position + QVector3D(0.f, 0.f, 0.1f);
    name_label.color = layer_color;
    name_label.char_height = label_height;
    labels.push_back(std::move(name_label));

    if (!layer.model_url.isEmpty()) {
      TextLabelInstance model_label;
      model_label.text = layer.model_url.toStdString();
      model_label.position = layer.position + QVector3D(0.f, 0.f, -label_height * 1.2f);
      model_label.color = QColor(148, 163, 184);
      model_label.char_height = label_height * 0.75f;
      labels.push_back(std::move(model_label));
    }
  }

  if (!labels.empty()) {
    drawLabelsOgreOrGl(context_, scene, name() + "/labels", labels);
  }
}

}  // namespace display
}  // namespace autoviz
