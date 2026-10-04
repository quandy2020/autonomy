/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file buffer.hpp
 * @brief Process-local TF buffer for Autoviz (automsgs message types).
 *
 * Extends @c tf2::BufferCore with Automsgs protobuf adapters, frame statistics
 * for the Transform Tree panel, and a process singleton via @ref Instance().
 *
 * @note Default lookup / canTransform timeout is @c 0 so display/UI paths never
 *       block the Qt thread. Pass a positive timeout only from worker code.
 *
 * @see Listener
 * @see ApplyTfMessageToBuffer
 * @see TfFrameStats
 */

#pragma once

#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>

#include "autoviz/transform/geometry_msgs/transform_stamped.h"
#include "autoviz/transform/tf2/buffer_core.h"

namespace autoviz {
namespace transform {

/**
 * @struct TfFrameStats
 * @brief Per-frame telemetry for the Transform Tree / TF diagnostics UI.
 */
struct TfFrameStats {
  /** Child frame id. */
  std::string frame_id;
  /** Parent frame id in the TF tree. */
  std::string parent_id;
  /** Authority / broadcaster string from @ref Buffer::setTransform. */
  std::string authority;
  /** Newest transform stamp in nanoseconds. */
  int64_t last_stamp_ns = 0;
  /** Oldest retained transform stamp in nanoseconds. */
  int64_t oldest_stamp_ns = 0;
  /** Estimated average publish rate for this frame (Hz). */
  double average_rate_hertz = 0.0;
  /** Span of buffered history in seconds. */
  double buffer_length_seconds = 0.0;
  /** Number of transforms received for this frame. */
  uint64_t transforms_received = 0;
  /** @c true if last updates were static (@c /tf_static). */
  bool is_static = false;
};

/**
 * @class Buffer
 * @brief Autoviz TF buffer singleton wrapping tf2 BufferCore with protobuf I/O.
 *
 * ## Responsibilities
 *
 * - Convert Automsgs @c TransformStamped ↔ internal tf2 geometry messages.
 * - Record @ref TfFrameStats on each @ref setTransform().
 * - Provide non-blocking lookups for Qt display threads (timeout default 0).
 *
 * @see Listener
 * @see tf2::BufferCore
 */
class Buffer : public tf2::BufferCore {
 public:
  /**
   * @brief Returns the process-local TF buffer singleton.
   * @return Non-owning pointer to the shared @ref Buffer (never @c nullptr).
   */
  static Buffer* Instance();

  /**
   * @brief Clears BufferCore storage and frame statistics.
   */
  void clear();

  /**
   * @brief Returns a snapshot of per-frame statistics for the UI.
   * @return Vector of @ref TfFrameStats (one entry per known frame).
   */
  std::vector<TfFrameStats> frameStats() const;

  /**
   * @brief Looks up the transform from @p source_frame to @p target_frame.
   *
   * @param target_frame Frame to express the transform in.
   * @param source_frame Frame being transformed.
   * @param time Query time (zero / latest semantics follow BufferCore).
   * @param timeout_second Max wait in seconds; default @c 0 (non-blocking UI).
   * @return Automsgs @c TransformStamped on success; throws tf2 exceptions on
   *         failure (same contract as BufferCore).
   *
   * @note Pass a positive timeout only from dedicated worker threads.
   */
  automsgs::msgs::geometry_msgs::TransformStamped lookupTransform(
      const std::string& target_frame, const std::string& source_frame,
      const automsgs::msgs::builtin_interfaces::Time& time,
      float timeout_second = 0.f) const;

  /**
   * @brief Tests whether a transform is available without throwing.
   *
   * @param target_frame Frame to express the transform in.
   * @param source_frame Frame being transformed.
   * @param time Query time.
   * @param timeout_second Max wait in seconds; default @c 0.
   * @param[out] errstr Optional human-readable reason when returning @c false.
   * @return @c true if lookup would succeed.
   */
  bool canTransform(const std::string& target_frame,
                    const std::string& source_frame,
                    const automsgs::msgs::builtin_interfaces::Time& time,
                    float timeout_second = 0.f,
                    std::string* errstr = nullptr) const;

  /**
   * @brief Inserts one stamped transform and updates frame statistics.
   *
   * @param transform Automsgs stamped transform (child → parent chain).
   * @param authority Broadcaster id stored in @ref TfFrameStats.
   * @param is_static When @c true, treated as static TF (no expiry).
   */
  void setTransform(
      const automsgs::msgs::geometry_msgs::TransformStamped& transform,
      const std::string& authority, bool is_static = false);

 private:
  /** @brief Private constructor; use @ref Instance(). */
  Buffer();

  /**
   * @brief Updates @c frame_stats_ for the child frame of @p transform.
   *
   * @param transform Newly applied stamped transform.
   * @param authority Broadcaster id.
   * @param is_static Static vs dynamic TF flag.
   */
  void recordFrameStats(
      const automsgs::msgs::geometry_msgs::TransformStamped& transform,
      const std::string& authority, bool is_static);

  /**
   * @brief Converts Automsgs protobuf transform to internal tf2 message.
   *
   * @param transform Automsgs stamped transform.
   * @return Internal @c geometry_msgs::TransformStamped for BufferCore.
   */
  static geometry_msgs::TransformStamped ToTf2Message(
      const automsgs::msgs::geometry_msgs::TransformStamped& transform);

  /**
   * @brief Converts internal tf2 message to Automsgs protobuf.
   *
   * @param transform Internal stamped transform from BufferCore.
   * @return Automsgs @c TransformStamped for Autoviz callers.
   */
  static automsgs::msgs::geometry_msgs::TransformStamped FromTf2Message(
      const geometry_msgs::TransformStamped& transform);

  /** Guards @c frame_stats_. */
  mutable std::mutex stats_mutex_;

  /** Per-frame telemetry keyed by frame id. */
  std::unordered_map<std::string, TfFrameStats> frame_stats_;
};

}  // namespace transform
}  // namespace autoviz
