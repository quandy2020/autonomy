/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file publish_tool_utils.hpp
 * @brief Shared helpers for Autolink publish tools (header fill, yaw quat, write).
 *
 * Used by @ref PublishPointTool, @ref NavGoalTool, @ref PoseEstimateTool and
 * similar one-shot publishers that emit stamped geometry messages.
 *
 * @see PublishPointTool
 * @see GoalPoseTool
 * @see NavGoalTool
 */

#pragma once

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "autolink/node/node.hpp"
#include "autolink/node/writer.hpp"
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/std_msgs/header.pb.h>

namespace autoviz {
namespace tools {

/**
 * @brief Fills @c std_msgs/Header with wall-clock stamp and @p frame_id.
 *
 * Stamp uses @c std::chrono::system_clock nanoseconds split into sec/nanosec
 * as expected by Automsgs protobuf headers.
 *
 * @param header Non-null destination header message.
 * @param frame_id TF / fixed frame id written to @c frame_id.
 */
inline void FillHeader(automsgs::msgs::std_msgs::Header* header,
                       const std::string& frame_id) {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now);
  const int64_t sec = ns.count() / 1000000000LL;
  const uint32_t nsec = static_cast<uint32_t>(ns.count() % 1000000000LL);
  header->mutable_stamp()->set_sec(static_cast<int32_t>(sec));
  header->mutable_stamp()->set_nanosec(nsec);
  header->set_frame_id(frame_id);
}

/**
 * @brief Sets a geometry_msgs quaternion from a yaw angle about +Z.
 *
 * Roll and pitch are zero; @p yaw is in radians (right-handed, Z-up).
 *
 * @param orientation Non-null quaternion message to overwrite.
 * @param yaw Yaw angle in radians.
 */
inline void SetYawQuaternion(
    automsgs::msgs::geometry_msgs::Quaternion* orientation,
    float yaw) {
  orientation->set_x(0.f);
  orientation->set_y(0.f);
  orientation->set_z(std::sin(yaw * 0.5f));
  orientation->set_w(std::cos(yaw * 0.5f));
}

/**
 * @brief Creates an Autolink writer for @p channel and writes one message.
 *
 * @tparam MessageT Protobuf message type supported by @c Node::CreateWriter.
 * @param node Shared Autolink node; must outlive the write.
 * @param channel Fully-qualified channel / topic name.
 * @param message Payload to publish (copied into a shared_ptr for Write).
 * @return @c true on successful @c Writer::Write; @c false if @p node is null,
 *         @p channel is empty, or writer creation / write fails.
 *
 * @note Each call creates a fresh writer; tools that publish frequently may
 *       cache their own @c Writer instead.
 */
template <typename MessageT>
bool PublishMessage(const std::shared_ptr<::autolink::Node>& node,
                    const std::string& channel, const MessageT& message) {
  if (node == nullptr || channel.empty()) {
    return false;
  }
  auto writer = node->CreateWriter<MessageT>(channel);
  if (writer == nullptr) {
    return false;
  }
  return writer->Write(std::make_shared<MessageT>(message));
}

}  // namespace tools
}  // namespace autoviz
