/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <string>

#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>

#include "autoviz/transform/geometry_msgs/transform_stamped.h"
#include "autoviz/transform/tf2/buffer_core.h"

namespace autoviz {
namespace transform {

/** Process-local TF buffer for Autoviz (automsgs message types). */
class Buffer : public tf2::BufferCore {
 public:
  static Buffer* Instance();

  void clear();

  automsgs::msgs::geometry_msgs::TransformStamped lookupTransform(
      const std::string& target_frame, const std::string& source_frame,
      const automsgs::msgs::builtin_interfaces::Time& time,
      float timeout_second = 0.01f) const;

  bool canTransform(const std::string& target_frame,
                    const std::string& source_frame,
                    const automsgs::msgs::builtin_interfaces::Time& time,
                    float timeout_second = 0.01f,
                    std::string* errstr = nullptr) const;

  void setTransform(
      const automsgs::msgs::geometry_msgs::TransformStamped& transform,
      const std::string& authority, bool is_static = false);

 private:
  Buffer();
  static geometry_msgs::TransformStamped ToTf2Message(
      const automsgs::msgs::geometry_msgs::TransformStamped& transform);
  static automsgs::msgs::geometry_msgs::TransformStamped FromTf2Message(
      const geometry_msgs::TransformStamped& transform);
};

}  // namespace transform
}  // namespace autoviz
