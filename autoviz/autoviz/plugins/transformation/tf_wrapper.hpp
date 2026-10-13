/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_default_plugins transformation::TFWrapper
 * (Bosch Software Innovations / Open Source Robotics Foundation).
 *****************************************************************************/

/**
 * @file tf_wrapper.hpp
 * @brief Thin adapter over Autoviz @c transform::Buffer (no ROS / tf2_ros).
 *
 * RViz @c TFWrapper owns a @c tf2_ros::Buffer + TransformListener. Autoviz
 * already fills a process-local @ref transform::Buffer via Autolink, so this
 * wrapper only delegates lookups and frame queries.
 *
 * @see TFFrameTransformer
 * @see transform::Buffer
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>

#include "autoviz/common/transformation/frame_transformer.hpp"

namespace autoviz {
namespace transform {
class Buffer;
}

namespace display {
namespace transformation {

/**
 * @class TFWrapper
 * @brief Non-owning facade around @ref transform::Buffer.
 */
class TFWrapper : public common::transformation::TransformationLibraryConnector {
 public:
  TFWrapper();
  explicit TFWrapper(transform::Buffer* buffer);
  ~TFWrapper() override = default;

  /** Binds (or rebinds) the shared Autoviz TF buffer. */
  void initialize(transform::Buffer* buffer);

  void clear();

  transform::Buffer* getBuffer() const { return buffer_; }

  void transform(const automsgs::msgs::geometry_msgs::PoseStamped& pose_in,
                 automsgs::msgs::geometry_msgs::PoseStamped* pose_out,
                 const std::string& target_frame) const;

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
                    std::string* error) const;

  bool canTransform(const std::string& target_frame,
                    const automsgs::msgs::builtin_interfaces::Time& target_time,
                    const std::string& source_frame,
                    const automsgs::msgs::builtin_interfaces::Time& source_time,
                    const std::string& fixed_frame, std::string* error) const;

  std::vector<std::string> getFrameStrings() const;
  bool frameExists(const std::string& frame) const;

 private:
  transform::Buffer* buffer_ = nullptr;
};

}  // namespace transformation
}  // namespace display
}  // namespace autoviz
