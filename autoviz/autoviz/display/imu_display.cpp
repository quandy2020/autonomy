/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/imu_display.hpp"

#include <algorithm>
#include <QtMath>

#include "autoviz/commsgs/time_utils.hpp"
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/ogre_colored_points_draw.hpp"
#include "autoviz/display/ogre_overlay_draw.hpp"
#include "autoviz/display/transform_utils.hpp"

namespace autoviz {
namespace display {
namespace {

QVector3D ProtoVector3(
    const automsgs::msgs::geometry_msgs::Vector3& vector) {
  return QVector3D(static_cast<float>(vector.x()), static_cast<float>(vector.y()),
                   static_cast<float>(vector.z()));
}

void yawPitchRollFromQuaternion(
    const automsgs::msgs::geometry_msgs::Quaternion& q, float* roll,
    float* pitch, float* yaw) {
  const double sinr_cosp = 2.0 * (q.w() * q.x() + q.y() * q.z());
  const double cosr_cosp = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
  const double sinp = 2.0 * (q.w() * q.y() - q.z() * q.x());
  const double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
  const double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  *roll = static_cast<float>(std::atan2(sinr_cosp, cosr_cosp));
  *pitch = static_cast<float>(std::asin(std::clamp(sinp, -1.0, 1.0)));
  *yaw = static_cast<float>(std::atan2(siny_cosp, cosy_cosp));
}

}  // namespace

ImuDisplay::ImuDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::sensor_msgs::Imu>(
          "Imu", std::move(channel), "automsgs.msgs.sensor_msgs.Imu") {
  setProperties({});
}

std::vector<common::DisplayPropertySpec> ImuDisplay::propertySpecs() const {
  return {{"accel_color", "Accel Color", "255;120;80"},
          {"gyro_color", "Gyro Color", "80;160;255"},
          {"axis_length", "Axis Length", "0.4"},
          {"accel_scale", "Accel Scale", "0.05"},
          {"gyro_scale", "Gyro Scale", "0.05"}};
}

void ImuDisplay::processMessage(const automsgs::msgs::sensor_msgs::Imu& message) {
  if (context_ == nullptr) {
    return;
  }
  const auto zero_time = autoviz::commsgs::ZeroTime();
  const std::string frame = message.header().frame_id().empty()
                                ? context_->fixed_frame
                                : message.header().frame_id();
  const QVector3D local_accel = ProtoVector3(message.linear_acceleration());
  const QVector3D local_gyro = ProtoVector3(message.angular_velocity());
  try {
    const auto tf = context_->tf_buffer->lookupTransform(context_->fixed_frame,
                                                          frame, zero_time);
    origin_ = transformPoint(tf, QVector3D(0.f, 0.f, 0.f));
    linear_accel_ = rotateVector(tf.transform().rotation(), local_accel);
    angular_vel_ = rotateVector(tf.transform().rotation(), local_gyro);
    yawPitchRollFromQuaternion(message.orientation(), &roll_, &pitch_, &yaw_);
  } catch (...) {
    origin_ = QVector3D(0.f, 0.f, 0.f);
    linear_accel_ = local_accel;
    angular_vel_ = local_gyro;
    yawPitchRollFromQuaternion(message.orientation(), &roll_, &pitch_, &yaw_);
  }
  have_imu_ = true;
  if (context_->request_redraw) {
    context_->request_redraw();
  }
}

void ImuDisplay::clearReceivedData() {
  have_imu_ = false;
}

void ImuDisplay::onDraw(rendering::SceneOverlay& scene) {
  if (!have_imu_) {
    return;
  }
  const QColor accel_color =
      common::ParseColorProperty(propertyValue("accel_color", "255;120;80"));
  const QColor gyro_color =
      common::ParseColorProperty(propertyValue("gyro_color", "80;160;255"));
  const float axis_length =
      common::ParseFloatProperty(propertyValue("axis_length", "0.4"), 0.4f);
  const float accel_scale =
      common::ParseFloatProperty(propertyValue("accel_scale", "0.05"), 0.05f);
  const float gyro_scale =
      common::ParseFloatProperty(propertyValue("gyro_scale", "0.05"), 0.05f);

  const QVector3D x_axis(axis_length * qCos(yaw_) * qCos(pitch_),
                         axis_length * qSin(yaw_) * qCos(pitch_),
                         axis_length * qSin(pitch_));
  const QVector3D y_axis(-axis_length * qSin(yaw_), axis_length * qCos(yaw_),
                         0.f);
  const QVector3D z_axis(0.f, 0.f, axis_length);
  drawLineSegmentsOgreOrGl(
      context_, scene, name() + "/axes",
      {{origin_, origin_ + x_axis, QColor(220, 80, 80)},
       {origin_, origin_ + y_axis, QColor(80, 220, 80)},
       {origin_, origin_ + z_axis, QColor(80, 120, 220)}});
  drawVectorArrowOgreOrGl(context_, scene, name() + "/accel", origin_,
                          linear_accel_, accel_color, 0.f, accel_scale);
  drawVectorArrowOgreOrGl(context_, scene, name() + "/gyro", origin_,
                          angular_vel_, gyro_color, 0.f, gyro_scale);
  drawColoredPointsOgreOrGl(context_, scene, name() + "/origin", typeId(), 4.f,
                            rendering::PointCloudStyle::kSquares,
                            {{origin_, QColor(255, 255, 255)}}, false);
}

}  // namespace display
}  // namespace autoviz
