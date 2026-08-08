/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>

#include <autolink/autolink.hpp>
#include <autolink/message/raw_message.hpp>
#include <autolink/proto/role_attributes.pb.h>
#include <autolink/time/rate.hpp>
#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/std_msgs/header.pb.h>

namespace autoviz::examples {

inline void StampHeader(automsgs::msgs::std_msgs::Header* h,
                        const std::string& frame = "map") {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  const auto ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
  h->set_frame_id(frame);
  h->mutable_stamp()->set_sec(static_cast<int32_t>(ns / 1000000000LL));
  h->mutable_stamp()->set_nanosec(static_cast<uint32_t>(ns % 1000000000LL));
}

inline double NowSec() {
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  return std::chrono::duration<double>(now).count();
}

inline void SetYawPose(automsgs::msgs::geometry_msgs::Pose* pose, double x,
                       double y, double yaw, double z = 0.05) {
  pose->mutable_position()->set_x(x);
  pose->mutable_position()->set_y(y);
  pose->mutable_position()->set_z(z);
  pose->mutable_orientation()->set_z(std::sin(yaw * 0.5));
  pose->mutable_orientation()->set_w(std::cos(yaw * 0.5));
}

inline double ParseRate(int argc, char** argv, double def = 10.0) {
  for (int i = 1; i + 1 < argc; ++i) {
    if (std::strcmp(argv[i], "--rate") == 0) return std::atof(argv[i + 1]);
  }
  return def;
}

inline bool ParseFlag(int argc, char** argv, const char* name) {
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], name) == 0) return true;
  }
  return false;
}

inline std::string ReadFile(const std::string& path) {
  std::ifstream in(path);
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

/** Resolve pr2_simple.urdf next to python tutorials. */
inline std::string ResolveUrdfPath() {
  if (const char* env = std::getenv("AUTOVIZ_EXAMPLES_DIR")) {
    return std::string(env) + "/python/urdf/pr2_simple.urdf";
  }
  // relative from cwd when run from autoviz root
  return "examples/python/urdf/pr2_simple.urdf";
}

template <typename MsgT>
std::shared_ptr<autolink::Writer<MsgT>> MakeWriter(
    const std::shared_ptr<autolink::Node>& node, const std::string& channel) {
  return node->CreateWriter<MsgT>(channel);
}

inline std::shared_ptr<autolink::Writer<autolink::message::RawMessage>>
MakeRawWriter(const std::shared_ptr<autolink::Node>& node,
              const std::string& channel, const std::string& message_type) {
  autolink::proto::RoleAttributes attr;
  attr.set_channel_name(channel);
  attr.set_message_type(message_type);
  return node->CreateWriter<autolink::message::RawMessage>(attr);
}

}  // namespace autoviz::examples
