/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_common/transformation/identity_frame_transformer.hpp
 * (Bosch Software Innovations).
 *****************************************************************************/

/**
 * @file identity_frame_transformer.hpp
 * @brief No-op transformer: all frames treated as coincident.
 */

#pragma once

#include <string>
#include <vector>

#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/quaternion.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>

#include "autoviz/common/transformation/frame_transformer.hpp"

namespace autoviz {
namespace common {
namespace transformation {

/**
 * @class IdentityFrameTransformer
 * @brief Identity TF backend (@ref buffer() is @c nullptr, @ref identityMode() @c true).
 */
class IdentityFrameTransformer : public FrameTransformer {
 public:
  PluginInfo info() const override;

  transform::Buffer* buffer() const override { return nullptr; }

  bool identityMode() const override { return true; }

  automsgs::msgs::geometry_msgs::PoseStamped transform(
      const automsgs::msgs::geometry_msgs::PoseStamped& pose_in,
      const std::string& target_frame) const;

  TransformationLibraryConnector::WeakPtr getConnector() const;

  bool frameHasProblems(const std::string& frame, std::string* error) const;

  automsgs::msgs::geometry_msgs::TransformStamped lookupTransform(
      const std::string& target_frame, const std::string& source_frame,
      const automsgs::msgs::builtin_interfaces::Time& time) const;

  automsgs::msgs::geometry_msgs::TransformStamped lookupTransform(
      const std::string& target_frame,
      const automsgs::msgs::builtin_interfaces::Time& target_time,
      const std::string& source_frame,
      const automsgs::msgs::builtin_interfaces::Time& source_time,
      const std::string& fixed_frame) const;

  bool canTransform(const std::string& target_frame,
                    const std::string& source_frame,
                    const automsgs::msgs::builtin_interfaces::Time& time,
                    std::string* error_msg = nullptr) const;

  bool canTransform(
      const std::string& target_frame,
      const automsgs::msgs::builtin_interfaces::Time& target_time,
      const std::string& source_frame,
      const automsgs::msgs::builtin_interfaces::Time& source_time,
      const std::string& fixed_frame,
      std::string* error_msg = nullptr) const;

  std::vector<std::string> getAllFrameNames() const;

 private:
  static bool quaternionIsValid(
      const automsgs::msgs::geometry_msgs::Quaternion& quaternion);
};

}  // namespace transformation
}  // namespace common
}  // namespace autoviz
