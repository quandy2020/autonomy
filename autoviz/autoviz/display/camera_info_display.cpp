/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/camera_info_display.hpp"

#include "autoviz/commsgs/time_utils.hpp"
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/camera_utils.hpp"
#include "autoviz/display/transform_utils.hpp"

namespace autoviz {
namespace display {

CameraInfoDisplay::CameraInfoDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::sensor_msgs::CameraInfo>(
          "CameraInfo", std::move(channel),
          "automsgs.msgs.sensor_msgs.CameraInfo") {
  setProperties({});
}

std::vector<common::DisplayPropertySpec> CameraInfoDisplay::propertySpecs() const {
  return {{"far_distance", "Far Clip Distance", "1.0", {}},
          {"show_edges", "Show Edges", "true", {}},
          {"show_side_edges", "Show Side Edges", "false", {}},
          {"color", "Color", "85;255;255", {}, common::DisplayPropertyKind::kColor},
          {"alpha", "Alpha", "0.5", {}}};
}

void CameraInfoDisplay::processMessage(
    const automsgs::msgs::sensor_msgs::CameraInfo& message) {
  camera_info_ = message;
  have_info_ = true;
  if (context_ != nullptr && context_->request_redraw) {
    context_->request_redraw();
  }
}

void CameraInfoDisplay::clearReceivedData() {
  have_info_ = false;
  camera_info_ = {};
}

void CameraInfoDisplay::onDraw(rendering::SceneOverlay& scene) {
  if (!have_info_ || context_ == nullptr || context_->tf_buffer == nullptr) {
    return;
  }

  const auto zero_time = autoviz::commsgs::ZeroTime();
  const std::string frame = camera_info_.header().frame_id().empty()
                                ? context_->fixed_frame
                                : camera_info_.header().frame_id();
  automsgs::msgs::geometry_msgs::TransformStamped tf;
  try {
    tf = context_->tf_buffer->lookupTransform(context_->fixed_frame, frame,
                                              zero_time);
  } catch (...) {
    return;
  }

  const float far_distance = common::ParseFloatProperty(
      propertyValue("far_distance", "1.0"), 1.f);
  const bool show_edges =
      common::ParseBoolProperty(propertyValue("show_edges", "true"), true);
  const bool show_side_edges = common::ParseBoolProperty(
      propertyValue("show_side_edges", "false"), false);
  QColor color =
      common::ParseColorProperty(propertyValue("color", "85;255;255"));
  const float alpha =
      common::ParseFloatProperty(propertyValue("alpha", "0.5"), 0.5f);
  color.setAlphaF(alpha);

  const QMatrix4x4 transform = opticalToWorld(camera_info_, transformToMatrix(tf));
  const float near_d = 0.05f;
  const auto segments =
      buildCameraFrustumSegments(camera_info_, near_d, far_distance);

  if (show_edges) {
    for (const auto& segment : segments) {
      scene.addLine(transform.map(segment.first), transform.map(segment.second),
                    color);
    }
  }

  if (show_side_edges && camera_info_.k_size() >= 9) {
    const float fx = static_cast<float>(camera_info_.k(0));
    const float fy = static_cast<float>(camera_info_.k(4));
    const float cx = static_cast<float>(camera_info_.k(2));
    const float cy = static_cast<float>(camera_info_.k(5));
    const float width = static_cast<float>(camera_info_.width());
    const float height = static_cast<float>(camera_info_.height());
    const auto corner_at = [&](float depth, float px, float py) {
      return QVector3D((px - cx) * depth / fx, (py - cy) * depth / fy, depth);
    };
    const QVector3D origin(0.f, 0.f, 0.f);
    const QVector3D far_tl = corner_at(far_distance, 0.f, 0.f);
    const QVector3D far_tr = corner_at(far_distance, width, 0.f);
    const QVector3D far_br = corner_at(far_distance, width, height);
    const QVector3D far_bl = corner_at(far_distance, 0.f, height);
    scene.addLine(transform.map(origin), transform.map(far_tl), color);
    scene.addLine(transform.map(origin), transform.map(far_tr), color);
    scene.addLine(transform.map(origin), transform.map(far_br), color);
    scene.addLine(transform.map(origin), transform.map(far_bl), color);
  }
}

}  // namespace display
}  // namespace autoviz
