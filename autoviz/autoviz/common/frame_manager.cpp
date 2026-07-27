/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/frame_manager.hpp"

#include <QQuaternion>
#include <QRegularExpression>

#include "autoviz/transform/buffer.hpp"

namespace autoviz {
namespace common {
namespace {

automsgs::msgs::builtin_interfaces::Time TimeAt(double sec) {
  automsgs::msgs::builtin_interfaces::Time time;
  const int64_t ns = static_cast<int64_t>(sec * 1e9);
  time.set_sec(static_cast<int32_t>(ns / 1000000000LL));
  time.set_nanosec(static_cast<uint32_t>(ns % 1000000000LL));
  return time;
}

QQuaternion FromProtoQuat(
    const automsgs::msgs::geometry_msgs::Quaternion& q) {
  return QQuaternion(q.w(), q.x(), q.y(), q.z());
}

}  // namespace

void FrameManager::setFixedFrame(const std::string& frame) {
  fixed_frame_ = frame;
}

void FrameManager::syncTime(double sec) { synced_time_sec_ = sec; }

double FrameManager::currentTimeSec() const {
  if (sync_mode_ == SyncMode::kOff) {
    return 0.0;
  }
  return synced_time_sec_;
}

bool FrameManager::getTransform(const std::string& frame, QVector3D* position,
                                QQuaternion* orientation) const {
  if (position == nullptr || orientation == nullptr) {
    return false;
  }
  if (frame.empty() || frame == fixed_frame_ || identity_mode_) {
    *position = QVector3D(0.f, 0.f, 0.f);
    *orientation = QQuaternion();
    return true;
  }
  if (buffer_ == nullptr) {
    return false;
  }
  const automsgs::msgs::builtin_interfaces::Time time = TimeAt(currentTimeSec());
  try {
    const automsgs::msgs::geometry_msgs::TransformStamped tf =
        buffer_->lookupTransform(fixed_frame_, frame, time);
    *position = QVector3D(tf.transform().translation().x(),
                          tf.transform().translation().y(),
                          tf.transform().translation().z());
    *orientation = FromProtoQuat(tf.transform().rotation());
    return true;
  } catch (...) {
    return false;
  }
}

bool FrameManager::transformHasProblems(const std::string& frame,
                                        std::string* error) const {
  if (identity_mode_) {
    return false;
  }
  if (buffer_ == nullptr) {
    if (error != nullptr) {
      *error = "TF buffer unavailable";
    }
    return true;
  }
  if (frame.empty() || frame == fixed_frame_) {
    return false;
  }
  std::string err;
  const automsgs::msgs::builtin_interfaces::Time time = TimeAt(currentTimeSec());
  if (!buffer_->canTransform(fixed_frame_, frame, time, 0.f, &err)) {
    if (error != nullptr) {
      *error = err.empty() ? "Transform unavailable" : err;
    }
    return true;
  }
  return false;
}

bool FrameManager::frameHasProblems(const std::string& frame,
                                    std::string* error) const {
  if (identity_mode_) {
    return false;
  }
  if (buffer_ == nullptr) {
    if (error != nullptr) {
      *error = "TF buffer unavailable";
    }
    return true;
  }
  const std::string frames_text = buffer_->allFramesAsString();
  const QRegularExpression pattern(
      QStringLiteral(R"(Frame (\S+) exists)"));
  QRegularExpressionMatchIterator it =
      pattern.globalMatch(QString::fromStdString(frames_text));
  while (it.hasNext()) {
    if (it.next().captured(1).toStdString() == frame) {
      return false;
    }
  }
  if (error != nullptr) {
    *error = "Frame does not exist";
  }
  return true;
}

std::vector<std::string> FrameManager::allFrameNames() const {
  std::vector<std::string> names;
  if (identity_mode_ || buffer_ == nullptr) {
    return names;
  }
  const QRegularExpression pattern(
      QStringLiteral(R"(Frame (\S+) exists)"));
  QRegularExpressionMatchIterator it = pattern.globalMatch(
      QString::fromStdString(buffer_->allFramesAsString()));
  while (it.hasNext()) {
    names.push_back(it.next().captured(1).toStdString());
  }
  return names;
}

void FrameManager::update() {
  // Per-frame cache hook; TF lookups remain in Buffer for now.
}

}  // namespace common
}  // namespace autoviz
