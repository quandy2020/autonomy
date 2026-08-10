/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/transform/buffer.hpp"

#include <chrono>
#include <mutex>
#include <thread>

#include <glog/logging.h>

#include "autoviz/commsgs/time_utils.hpp"
#include "autoviz/transform/tf2/exceptions.h"
#include <automsgs/msgs/time_utils.hpp>

namespace autoviz {
namespace transform {
namespace {

constexpr float kSecondToNanoFactor = 1e9f;

bool IsFutureExtrapolation(const std::string& err) {
  return err.find("extrapolation into the future") != std::string::npos;
}

}  // namespace

Buffer::Buffer() : tf2::BufferCore() {}

Buffer* Buffer::Instance() {
  static Buffer instance;
  return &instance;
}

void Buffer::clear() {
  {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    frame_stats_.clear();
  }
  tf2::BufferCore::clear();
}

geometry_msgs::TransformStamped Buffer::ToTf2Message(
    const automsgs::msgs::geometry_msgs::TransformStamped& trans) {
  geometry_msgs::TransformStamped geo_msg;
  geo_msg.header.stamp = automsgs::msgs::builtin_interfaces::TimeToNanoseconds(
      trans.header().stamp());
  geo_msg.header.frame_id = trans.header().frame_id();
  geo_msg.child_frame_id = trans.child_frame_id();
  geo_msg.transform.translation.x = trans.transform().translation().x();
  geo_msg.transform.translation.y = trans.transform().translation().y();
  geo_msg.transform.translation.z = trans.transform().translation().z();
  geo_msg.transform.rotation.x = trans.transform().rotation().x();
  geo_msg.transform.rotation.y = trans.transform().rotation().y();
  geo_msg.transform.rotation.z = trans.transform().rotation().z();
  geo_msg.transform.rotation.w = trans.transform().rotation().w();
  return geo_msg;
}

automsgs::msgs::geometry_msgs::TransformStamped Buffer::FromTf2Message(
    const geometry_msgs::TransformStamped& tf2_trans_stamped) {
  automsgs::msgs::geometry_msgs::TransformStamped trans_stamped;
  trans_stamped.mutable_header()->mutable_stamp()->set_sec(
      static_cast<int32_t>(tf2_trans_stamped.header.stamp / 1'000'000'000ULL));
  trans_stamped.mutable_header()->mutable_stamp()->set_nanosec(
      static_cast<uint32_t>(tf2_trans_stamped.header.stamp % 1'000'000'000ULL));
  trans_stamped.mutable_header()->set_frame_id(tf2_trans_stamped.header.frame_id);
  trans_stamped.set_child_frame_id(tf2_trans_stamped.child_frame_id);
  trans_stamped.mutable_transform()->mutable_translation()->set_x(
      tf2_trans_stamped.transform.translation.x);
  trans_stamped.mutable_transform()->mutable_translation()->set_y(
      tf2_trans_stamped.transform.translation.y);
  trans_stamped.mutable_transform()->mutable_translation()->set_z(
      tf2_trans_stamped.transform.translation.z);
  trans_stamped.mutable_transform()->mutable_rotation()->set_x(
      tf2_trans_stamped.transform.rotation.x);
  trans_stamped.mutable_transform()->mutable_rotation()->set_y(
      tf2_trans_stamped.transform.rotation.y);
  trans_stamped.mutable_transform()->mutable_rotation()->set_z(
      tf2_trans_stamped.transform.rotation.z);
  trans_stamped.mutable_transform()->mutable_rotation()->set_w(
      tf2_trans_stamped.transform.rotation.w);
  return trans_stamped;
}

void Buffer::recordFrameStats(
    const automsgs::msgs::geometry_msgs::TransformStamped& transform,
    const std::string& authority, bool is_static) {
  const std::string child = transform.child_frame_id();
  if (child.empty()) {
    return;
  }
  std::lock_guard<std::mutex> lock(stats_mutex_);
  TfFrameStats& stats = frame_stats_[child];
  stats.frame_id = child;
  stats.parent_id = transform.header().frame_id();
  stats.authority = authority;
  stats.last_stamp_ns = static_cast<int64_t>(
      automsgs::msgs::builtin_interfaces::TimeToNanoseconds(transform.header().stamp()));
  stats.transforms_received += 1;
  stats.is_static = is_static || stats.is_static;
}

std::vector<TfFrameStats> Buffer::frameStats() const {
  std::vector<std::string> frame_ids;
  _getFrameStrings(frame_ids);

  std::lock_guard<std::mutex> lock(stats_mutex_);
  std::vector<TfFrameStats> out;
  out.reserve(frame_ids.size());
  for (const std::string& frame_id : frame_ids) {
    if (frame_id == "NO_PARENT") {
      continue;
    }
    TfFrameStats stats;
    const auto it = frame_stats_.find(frame_id);
    if (it != frame_stats_.end()) {
      stats = it->second;
    } else {
      stats.frame_id = frame_id;
    }
    // recordFrameStats already fills parent_id; only resolve when missing.
    if (stats.parent_id.empty() || stats.parent_id == "NO_PARENT") {
      std::string parent;
      if (_getParent(frame_id, 0, parent)) {
        stats.parent_id = parent;
      }
    }
    out.push_back(std::move(stats));
  }
  return out;
}

void Buffer::setTransform(
    const automsgs::msgs::geometry_msgs::TransformStamped& transform,
    const std::string& authority, bool is_static) {
  try {
    recordFrameStats(transform, authority, is_static);
    tf2::BufferCore::setTransform(ToTf2Message(transform), authority, is_static);
  } catch (const tf2::TransformException& ex) {
    LOG(WARNING) << "Failed to apply transform [" << transform.header().frame_id()
                 << " -> " << transform.child_frame_id() << "]: " << ex.what();
  }
}

automsgs::msgs::geometry_msgs::TransformStamped Buffer::lookupTransform(
    const std::string& target_frame, const std::string& source_frame,
    const automsgs::msgs::builtin_interfaces::Time& time,
    const float timeout_second) const {
  if (target_frame == source_frame) {
    automsgs::msgs::geometry_msgs::TransformStamped out;
    *out.mutable_header()->mutable_stamp() = time;
    out.mutable_header()->set_frame_id(target_frame);
    out.set_child_frame_id(source_frame);
    out.mutable_transform()->mutable_rotation()->set_w(1.0);
    return out;
  }

  std::string err;
  if (!const_cast<Buffer*>(this)->canTransform(target_frame, source_frame, time,
                                               timeout_second, &err)) {
    throw tf2::TimeoutException("TF lookupTransform timeout: " + err);
  }

  uint64_t tf2_time_ns =
      automsgs::msgs::builtin_interfaces::TimeToNanoseconds(time);
  if (tf2_time_ns != 0 && IsFutureExtrapolation(err)) {
    tf2_time_ns = 0ULL;
  }
  const auto tf2_transform =
      tf2::BufferCore::lookupTransform(target_frame, source_frame, tf2_time_ns);
  return FromTf2Message(tf2_transform);
}

bool Buffer::canTransform(const std::string& target_frame,
                            const std::string& source_frame,
                            const automsgs::msgs::builtin_interfaces::Time& time,
                            const float timeout_second,
                            std::string* errstr) const {
  const uint64_t requested_ns =
      automsgs::msgs::builtin_interfaces::TimeToNanoseconds(time);
  const auto tryOnce = [&](std::string* err) -> bool {
    const bool ok = tf2::BufferCore::canTransform(target_frame, source_frame,
                                                  requested_ns, err);
    if (ok) {
      return true;
    }
    if (requested_ns != 0 && err != nullptr && IsFutureExtrapolation(*err)) {
      std::string latest_err;
      return tf2::BufferCore::canTransform(target_frame, source_frame, 0ULL,
                                           &latest_err);
    }
    return false;
  };

  // Non-blocking (display / UI thread): one attempt, never sleep.
  if (timeout_second <= 0.f) {
    std::string local_err;
    std::string* err = errstr != nullptr ? errstr : &local_err;
    return tryOnce(err);
  }

  const uint64_t timeout_ns =
      static_cast<uint64_t>(timeout_second * kSecondToNanoFactor);
  const auto start = std::chrono::steady_clock::now();
  const auto deadline = start + std::chrono::nanoseconds(timeout_ns);
  while (std::chrono::steady_clock::now() < deadline) {
    if (errstr != nullptr) {
      errstr->clear();
    }
    std::string local_err;
    std::string* err = errstr != nullptr ? errstr : &local_err;
    if (tryOnce(err)) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(3));
  }
  if (errstr != nullptr) {
    *errstr = *errstr + ":timeout";
  }
  return false;
}

}  // namespace transform
}  // namespace autoviz
