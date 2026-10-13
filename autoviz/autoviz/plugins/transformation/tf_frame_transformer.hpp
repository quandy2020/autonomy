/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_default_plugins transformation::TFFrameTransformer
 * (Bosch Software Innovations / Open Source Robotics Foundation).
 *****************************************************************************/

/**
 * @file tf_frame_transformer.hpp
 * @brief Default @ref common::transformation::FrameTransformer backed by Autoviz TF buffer.
 *
 * Drop-in for RViz @c TFFrameTransformer without ROS node / tf2_ros listener.
 * Registered as @c "autoviz/AutolinkTf" by @ref TransformationManager.
 *
 * @see TFWrapper
 * @see TransformerGuard
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>

#include "autoviz/common/transformation/frame_transformer.hpp"
#include "autoviz/plugins/transformation/tf_wrapper.hpp"

namespace autoviz {
namespace display {
namespace transformation {

/**
 * @class TFFrameTransformer
 * @brief FrameTransformer implementation using @ref TFWrapper.
 */
class TFFrameTransformer : public common::transformation::FrameTransformer {
 public:
  TFFrameTransformer();
  explicit TFFrameTransformer(std::shared_ptr<TFWrapper> wrapper);
  explicit TFFrameTransformer(transform::Buffer* buffer);
  ~TFFrameTransformer() override = default;

  common::PluginInfo info() const override;
  transform::Buffer* buffer() const override;

  /** Binds the shared buffer (RViz @c initialize without ROS clock/node). */
  void initialize(transform::Buffer* buffer);

  void clear();

  std::vector<std::string> getAllFrameNames() const;

  automsgs::msgs::geometry_msgs::PoseStamped transform(
      const automsgs::msgs::geometry_msgs::PoseStamped& pose_in,
      const std::string& target_frame) const;

  bool frameHasProblems(const std::string& frame, std::string* error) const;

  std::weak_ptr<common::transformation::TransformationLibraryConnector>
  getConnector() const;

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
                    std::string* error = nullptr) const;

  bool canTransform(const std::string& target_frame,
                    const automsgs::msgs::builtin_interfaces::Time& target_time,
                    const std::string& source_frame,
                    const automsgs::msgs::builtin_interfaces::Time& source_time,
                    const std::string& fixed_frame,
                    std::string* error = nullptr) const;

 private:
  [[noreturn]] static void RethrowAsFrameTransformerException(
      const char* prefix, const std::exception& exception);

  std::shared_ptr<TFWrapper> tf_wrapper_;
};

}  // namespace transformation
}  // namespace display
}  // namespace autoviz
