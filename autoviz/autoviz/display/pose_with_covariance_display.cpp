/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/pose_with_covariance_display.hpp"

#include <QtMath>

#include "autoviz/commsgs/time_utils.hpp"
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/ogre_colored_points_draw.hpp"
#include "autoviz/display/ogre_overlay_draw.hpp"
#include "autoviz/display/transform_utils.hpp"

namespace autoviz {
namespace display {
namespace {

float yawFromQuaternion(
    const automsgs::msgs::geometry_msgs::Quaternion& q) {
  const double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
  const double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  return static_cast<float>(std::atan2(siny_cosp, cosy_cosp));
}

QQuaternion ProtoQuaternion(
    const automsgs::msgs::geometry_msgs::Quaternion& q) {
  return QQuaternion(static_cast<float>(q.w()), static_cast<float>(q.x()),
                     static_cast<float>(q.y()), static_cast<float>(q.z()));
}

}  // namespace

PoseWithCovarianceDisplay::PoseWithCovarianceDisplay(std::string channel)
    : ChannelDisplay<
          automsgs::msgs::geometry_msgs::PoseWithCovarianceStamped>(
          "PoseWithCovariance", std::move(channel),
          "automsgs.msgs.geometry_msgs.PoseWithCovarianceStamped") {
  setProperties({});
}

std::vector<common::DisplayPropertySpec>
PoseWithCovarianceDisplay::propertySpecs() const {
  return {{"color", "Color", "255;25;0"},
          {"axis_length", "Axis Length", "1.0"},
          {"covariance_scale", "Covariance Scale", "1.0"},
          {"orientation_covariance_scale", "Orientation Covariance Scale",
           "0.1"},
          {"orientation_covariance_offset", "Orientation Covariance Offset",
           "0.1"},
          {"show_covariance", "Show Covariance", "true"}};
}

void PoseWithCovarianceDisplay::processMessage(
    const automsgs::msgs::geometry_msgs::PoseWithCovarianceStamped& message) {
  if (context_ == nullptr) {
    return;
  }
  const auto zero_time = autoviz::commsgs::ZeroTime();
  const std::string frame = message.header().frame_id().empty()
                                ? context_->fixed_frame
                                : message.header().frame_id();
  const auto& stamped_pose = message.pose().pose();
  const auto& pose = stamped_pose.pose();
  pose_orientation_ = ProtoQuaternion(pose.orientation());
  yaw_ = yawFromQuaternion(pose.orientation());
  covariance_.fill(0.0);
  const auto& cov = message.pose().covariance();
  for (int i = 0; i < 36 && i < cov.size(); ++i) {
    covariance_[static_cast<std::size_t>(i)] = cov.Get(i);
  }
  try {
    const auto tf = context_->tf_buffer->lookupTransform(context_->fixed_frame,
                                                          frame, zero_time);
    const QVector3D local(static_cast<float>(pose.position().x()),
                          static_cast<float>(pose.position().y()),
                          static_cast<float>(pose.position().z()));
    position_ = transformPoint(tf, local);
    const QMatrix3x3 rot = transformToMatrix(tf).toGenericMatrix<3, 3>();
    frame_orientation_ = QQuaternion::fromRotationMatrix(rot).normalized();
  } catch (...) {
    position_ = QVector3D(static_cast<float>(pose.position().x()),
                          static_cast<float>(pose.position().y()),
                          static_cast<float>(pose.position().z()));
    frame_orientation_ = pose_orientation_;
  }
  have_pose_ = true;
  if (context_->request_redraw) {
    context_->request_redraw();
  }
}

void PoseWithCovarianceDisplay::clearReceivedData() {
  have_pose_ = false;
  position_ = QVector3D();
  yaw_ = 0.f;
}

void PoseWithCovarianceDisplay::onDraw(rendering::SceneOverlay& scene) {
  if (!have_pose_) {
    return;
  }
  const QColor color =
      common::ParseColorProperty(propertyValue("color", "255;25;0"));
  const float len =
      common::ParseFloatProperty(propertyValue("axis_length", "1.0"), 1.f);
  const float cov_scale = common::ParseFloatProperty(
      propertyValue("covariance_scale", "1.0"), 1.f);
  const float orientation_scale = common::ParseFloatProperty(
      propertyValue("orientation_covariance_scale", "0.1"), 0.1f);
  const float orientation_offset = common::ParseFloatProperty(
      propertyValue("orientation_covariance_offset", "0.1"), 0.1f);
  const bool show_covariance = common::ParseBoolProperty(
      propertyValue("show_covariance", "true"), true);

  const QVector3D heading(len * qCos(yaw_), len * qSin(yaw_), 0.f);
  drawArrowOgreOrGl(context_, scene, name() + "/heading", position_,
                    position_ + heading, color);
  drawLineSegmentsOgreOrGl(
      context_, scene, name() + "/axes",
      {{position_, position_ + heading, color},
       {position_, position_ + QVector3D(0.f, len * 0.5f, 0.f),
        QColor(60, 220, 60)}});
  drawColoredPointsOgreOrGl(context_, scene, name() + "/origin", typeId(), 4.f,
                            rendering::PointCloudStyle::kSquares,
                            {{position_, color}}, false);
  drawCovarianceOgreOrGl(context_, scene, name() + "/covariance", position_,
                         pose_orientation_, frame_orientation_, covariance_,
                         color, cov_scale, orientation_scale,
                         orientation_offset, show_covariance);
}

}  // namespace display
}  // namespace autoviz
