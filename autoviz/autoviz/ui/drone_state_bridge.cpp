/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/drone_state_bridge.hpp"

#include <algorithm>
#include <cmath>

#include <QQuaternion>

#include "autoviz/commsgs/time_utils.hpp"
#include "autoviz/qml/drone_vehicle_state.hpp"
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

}  // namespace

void DroneStateBridge::updateFromTf(const std::string& fixed_frame,
                                    transform::Buffer* tf_buffer,
                                    qml_drone::DroneVehicleState* state) {
  if (state == nullptr || tf_buffer == nullptr || source_frame_.empty()) {
    if (state != nullptr) {
      state->setValid(false);
    }
    return;
  }

  const auto zero_time = automsgs::msgs::ZeroTime();
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

    // Match QGC scene scale (×10) for F450 mesh coordinates.
    state->setX(relative.x() * 10.0);
    state->setY(relative.y() * 10.0);
    state->setZ(relative.z() * 10.0);
    state->setRoll(roll);
    state->setPitch(pitch);
    state->setYaw(yaw);
    state->setFlightMode(flight_mode_when_valid_);
    state->setLabel(QString::fromStdString(source_frame_));
    state->setValid(true);
  } catch (...) {
    state->setValid(false);
    state->setFlightMode(0);
  }
}

}  // namespace autoviz
