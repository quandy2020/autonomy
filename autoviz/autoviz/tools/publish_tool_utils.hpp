/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

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

inline void SetYawQuaternion(
    automsgs::msgs::geometry_msgs::Quaternion* orientation,
    float yaw) {
  orientation->set_x(0.f);
  orientation->set_y(0.f);
  orientation->set_z(std::sin(yaw * 0.5f));
  orientation->set_w(std::cos(yaw * 0.5f));
}

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
