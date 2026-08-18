/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

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

struct TfFrameStats {
  std::string frame_id;
  std::string parent_id;
  std::string authority;
  int64_t last_stamp_ns = 0;
  int64_t oldest_stamp_ns = 0;
  double average_rate_hertz = 0.0;
  double buffer_length_seconds = 0.0;
  uint64_t transforms_received = 0;
  bool is_static = false;
};

/** Process-local TF buffer for Autoviz (automsgs message types). */
class Buffer : public tf2::BufferCore {
 public:
  static Buffer* Instance();

  void clear();

  std::vector<TfFrameStats> frameStats() const;

  /** Default timeout is 0: display/UI paths must not block the Qt thread.
   *  Pass a positive timeout only from dedicated worker code. */
  automsgs::msgs::geometry_msgs::TransformStamped lookupTransform(
      const std::string& target_frame, const std::string& source_frame,
      const automsgs::msgs::builtin_interfaces::Time& time,
      float timeout_second = 0.f) const;

  bool canTransform(const std::string& target_frame,
                    const std::string& source_frame,
                    const automsgs::msgs::builtin_interfaces::Time& time,
                    float timeout_second = 0.f,
                    std::string* errstr = nullptr) const;

  void setTransform(
      const automsgs::msgs::geometry_msgs::TransformStamped& transform,
      const std::string& authority, bool is_static = false);

 private:
  Buffer();
  void recordFrameStats(
      const automsgs::msgs::geometry_msgs::TransformStamped& transform,
      const std::string& authority, bool is_static);

  static geometry_msgs::TransformStamped ToTf2Message(
      const automsgs::msgs::geometry_msgs::TransformStamped& transform);
  static automsgs::msgs::geometry_msgs::TransformStamped FromTf2Message(
      const geometry_msgs::TransformStamped& transform);

  mutable std::mutex stats_mutex_;
  std::unordered_map<std::string, TfFrameStats> frame_stats_;
};

}  // namespace transform
}  // namespace autoviz
