/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/vehicle_state_bridge.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <QQuaternion>

#include "autoviz/commsgs/time_utils.hpp"
#include "autoviz/qml/vehicle_state.hpp"
#include "autoviz/transform/buffer.hpp"

#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>

namespace autoviz {
namespace {

void quaternionToRpy(const QQuaternion& q, double* roll, double* pitch,
                     double* yaw) {
  const double sinr_cosp = 2.0 * (q.scalar() * q.x() + q.y() * q.z());
  const double cosr_cosp = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
  const double sinp = 2.0 * (q.scalar() * q.y() - q.z() * q.x());
  const double siny_cosp = 2.0 * (q.scalar() * q.z() + q.x() * q.y());
  const double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  *roll = std::atan2(sinr_cosp, cosr_cosp) * 180.0 / M_PI;
  *pitch = std::asin(std::clamp(sinp, -1.0, 1.0)) * 180.0 / M_PI;
  *yaw = std::atan2(siny_cosp, cosy_cosp) * 180.0 / M_PI;
}

QVector3D transformPoint(
    const automsgs::msgs::geometry_msgs::TransformStamped& tf,
    const QVector3D& point) {
  const auto& t = tf.transform().translation();
  const auto& q = tf.transform().rotation();
  const QQuaternion rotation(q.w(), q.x(), q.y(), q.z());
  return rotation.rotatedVector(point) +
         QVector3D(static_cast<float>(t.x()), static_cast<float>(t.y()),
                   static_cast<float>(t.z()));
}

double normalizeYawDeltaDegrees(double current, double previous) {
  double delta = current - previous;
  while (delta > 180.0) {
    delta -= 360.0;
  }
  while (delta < -180.0) {
    delta += 360.0;
  }
  return delta;
}

int motionModeFromSpeed(qml_vehicle::VehicleModelType type, double linear_mps,
                        double angular_dps) {
  const bool is_ground =
      type == qml_vehicle::VehicleModelType::kGroundDiffDrive ||
      type == qml_vehicle::VehicleModelType::kGroundAckermann;
  const double linear_threshold = is_ground ? 0.02 : 0.05;
  const double angular_threshold = is_ground ? 3.0 : 5.0;
  if (std::abs(linear_mps) < linear_threshold &&
      std::abs(angular_dps) < angular_threshold) {
    return 0;
  }
  const double fast_linear = is_ground ? 0.25 : 0.4;
  const double fast_angular = is_ground ? 25.0 : 35.0;
  if (std::abs(linear_mps) >= fast_linear ||
      std::abs(angular_dps) >= fast_angular) {
    return 2;
  }
  return 1;
}

}  // namespace

double VehicleStateBridge::sceneScaleForModel(
    qml_vehicle::VehicleModelType type) {
  switch (type) {
    case qml_vehicle::VehicleModelType::kDroneSimple:
    case qml_vehicle::VehicleModelType::kDroneF450:
      return 10.0;
    case qml_vehicle::VehicleModelType::kGroundDiffDrive:
    case qml_vehicle::VehicleModelType::kGroundAckermann:
    default:
      return 1.0;
  }
}

void VehicleStateBridge::resetOrigin() {
  have_origin_ = false;
  have_motion_sample_ = false;
}

void VehicleStateBridge::updateFromTf(const std::string& fixed_frame,
                                      transform::Buffer* tf_buffer,
                                      qml_vehicle::VehicleState* state) {
  if (state == nullptr || tf_buffer == nullptr || source_frame_.empty()) {
    if (state != nullptr) {
      state->setValid(false);
      state->setLinearSpeed(0.0);
      state->setAngularSpeed(0.0);
    }
    return;
  }

  const double scale = sceneScaleForModel(state->modelTypeEnum());
  state->setSceneScale(scale);

  const auto zero_time = autoviz::commsgs::ZeroTime();
  try {
    const auto tf = tf_buffer->lookupTransform(fixed_frame, source_frame_,
                                               zero_time);
    const QVector3D world = transformPoint(tf, QVector3D(0.f, 0.f, 0.f));
    if (!have_origin_) {
      origin_ = world;
      have_origin_ = true;
    }
    const QVector3D relative = world - origin_;

    const auto& q = tf.transform().rotation();
    const QQuaternion rotation(q.w(), q.x(), q.y(), q.z());
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    quaternionToRpy(rotation.normalized(), &roll, &pitch, &yaw);

    const double now_s =
        std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count();
    double linear_mps = 0.0;
    double angular_dps = 0.0;
    if (have_motion_sample_ && now_s > last_sample_time_s_) {
      const double dt = now_s - last_sample_time_s_;
      if (dt > 1e-3) {
        const double dx = relative.x() - last_relative_.x();
        const double dy = relative.y() - last_relative_.y();
        linear_mps = std::sqrt(dx * dx + dy * dy) / dt;
        angular_dps =
            normalizeYawDeltaDegrees(yaw, last_yaw_deg_) / dt;
      }
    }
    last_relative_ = relative;
    last_yaw_deg_ = yaw;
    last_sample_time_s_ = now_s;
    have_motion_sample_ = true;

    state->setX(relative.x() * scale);
    state->setY(relative.y() * scale);
    state->setZ(relative.z() * scale);
    state->setRoll(roll);
    state->setPitch(pitch);
    state->setYaw(yaw);
    state->setLinearSpeed(linear_mps);
    state->setAngularSpeed(angular_dps);
    state->setMotionMode(
        motionModeFromSpeed(state->modelTypeEnum(), linear_mps, angular_dps));
    state->setLabel(QString::fromStdString(source_frame_));
    state->setValid(true);
  } catch (...) {
    state->setValid(false);
    state->setMotionMode(0);
    state->setLinearSpeed(0.0);
    state->setAngularSpeed(0.0);
    have_motion_sample_ = false;
  }
}

}  // namespace autoviz
