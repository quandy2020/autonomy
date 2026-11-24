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

  // New transforms can arrive between display updates; dropping the cache on a
  // revision bump keeps results fresh without a per-call lookup.
  const std::uint64_t revision = buffer_->revision();
  if (revision != cache_revision_) {
    pose_cache_.clear();
    cache_revision_ = revision;
  }
  const auto cached = pose_cache_.find(frame);
  if (cached != pose_cache_.end()) {
    if (cached->second.ok) {
      *position = cached->second.position;
      *orientation = cached->second.orientation;
    }
    return cached->second.ok;
  }

  const double lookup_time =
      std::isnan(cache_time_sec_) ? currentTimeSec() : cache_time_sec_;
  const automsgs::msgs::builtin_interfaces::Time time = TimeAt(lookup_time);
  CachedPose pose;
  try {
    const automsgs::msgs::geometry_msgs::TransformStamped tf =
        buffer_->lookupTransform(fixed_frame_, frame, time);
    pose.ok = true;
    pose.position = QVector3D(tf.transform().translation().x(),
                              tf.transform().translation().y(),
                              tf.transform().translation().z());
    pose.orientation = FromProtoQuat(tf.transform().rotation());
  } catch (...) {
    pose.ok = false;
  }
  pose_cache_.emplace(frame, pose);
  if (!pose.ok) {
    return false;
  }
  *position = pose.position;
  *orientation = pose.orientation;
  return true;
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
  // Freeze one lookup timestamp per tick and drop last tick's poses.
  cache_time_sec_ = currentTimeSec();
  pose_cache_.clear();
  if (buffer_ != nullptr) {
    cache_revision_ = buffer_->revision();
  }
}

}  // namespace common
}  // namespace autoviz
