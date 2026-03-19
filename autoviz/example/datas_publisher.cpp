/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <signal.h>
#include <unistd.h>

#include <chrono>
#include <filesystem>
#include <string>
#include <thread>

#include "autolink/autolink.hpp"
#include "autolink/common/log.hpp"
#include "autolink/message/message_traits.hpp"
#include "autolink/proto/role_attributes.pb.h"
#include "automsgs/msgs/builtin_interfaces/time.pb.h"
#include "automsgs/msgs/sensor_msgs/imu.pb.h"
#include "automsgs/msgs/std_msgs/header.pb.h"
#include "automsgs/msgs/std_msgs/string.pb.h"

namespace autoviz {
namespace example {
namespace {

using autolink::Rate;
using autolink::Time;

bool g_running = true;

void SignalHandler(int) {
  g_running = false;
}

automsgs::msgs::builtin_interfaces::Time NowProto() {
  const int64_t nanos = Time::Now().ToNanosecond();
  automsgs::msgs::builtin_interfaces::Time t;
  t.set_sec(static_cast<int32_t>(nanos / 1000000000LL));
  t.set_nanosec(static_cast<uint32_t>(nanos % 1000000000ULL));
  return t;
}

automsgs::msgs::std_msgs::Header MakeHeader(const std::string& frame_id) {
  automsgs::msgs::std_msgs::Header h;
  *h.mutable_stamp() = NowProto();
  h.set_frame_id(frame_id);
  return h;
}

std::string GuessAutolinkWorkRoot() {
  namespace fs = std::filesystem;
  const fs::path cwd = fs::current_path();
  const fs::path conf_rel = fs::path("conf") / "autolink.pb.conf";
  fs::path base = cwd;
  for (int depth = 0; depth < 10; ++depth) {
    const fs::path conf = base / "src" / "autonomy" / "autolink" / "autolink" / conf_rel;
    std::error_code ec;
    if (fs::exists(conf, ec) && !ec) {
      return conf.parent_path().parent_path().string();
    }
    if (!base.has_parent_path()) {
      break;
    }
    base = base.parent_path();
  }
  return "";
}

}  // namespace

void Run() {
  signal(SIGINT, SignalHandler);
  signal(SIGTERM, SignalHandler);

  auto node = autolink::CreateNode("autoviz_datas_publisher_" + std::to_string(getpid()));
  if (!node) {
    AERROR << "Failed to create autolink node";
    return;
  }

  // /autoviz/example/text
  autolink::proto::RoleAttributes text_attr;
  text_attr.set_channel_name("/autoviz/example/text");
  text_attr.set_message_type(
      autolink::message::MessageType<automsgs::msgs::std_msgs::String>());
  auto text_writer = node->CreateWriter<automsgs::msgs::std_msgs::String>(text_attr);

  // /autoviz/example/imu
  autolink::proto::RoleAttributes imu_attr;
  imu_attr.set_channel_name("/autoviz/example/imu");
  imu_attr.set_message_type(
      autolink::message::MessageType<automsgs::msgs::sensor_msgs::Imu>());
  auto imu_writer = node->CreateWriter<automsgs::msgs::sensor_msgs::Imu>(imu_attr);

  if (!text_writer || !imu_writer) {
    AERROR << "Failed to create one or more writers";
    return;
  }

  AINFO << "autoviz datas_publisher started";
  AINFO << "  publish: /autoviz/example/text (std_msgs/String)";
  AINFO << "  publish: /autoviz/example/imu  (sensor_msgs/Imu)";

  uint64_t seq = 0;
  Rate loop_rate(10.0);  // 10 Hz
  while (g_running && autolink::OK()) {
    // std_msgs/String
    automsgs::msgs::std_msgs::String text_msg;
    text_msg.set_data("autoviz hello, seq=" + std::to_string(seq));
    text_writer->Write(text_msg);

    // sensor_msgs/Imu
    automsgs::msgs::sensor_msgs::Imu imu_msg;
    *imu_msg.mutable_header() = MakeHeader("base_link");
    imu_msg.mutable_orientation()->set_w(1.0);
    imu_msg.mutable_angular_velocity()->set_x(0.01 * static_cast<double>(seq));
    imu_msg.mutable_angular_velocity()->set_y(0.0);
    imu_msg.mutable_angular_velocity()->set_z(0.1);
    imu_msg.mutable_linear_acceleration()->set_x(0.0);
    imu_msg.mutable_linear_acceleration()->set_y(0.0);
    imu_msg.mutable_linear_acceleration()->set_z(9.81);
    imu_writer->Write(imu_msg);

    AINFO << "Published text+imu message, seq=" << seq;

    ++seq;
    loop_rate.Sleep();
  }

  AINFO << "autoviz datas_publisher stopped";
}

}  // namespace example
}  // namespace autoviz

int main(int argc, char** argv) {
  (void)argc;
  if (std::getenv("AUTOLINK_PATH") == nullptr) {
    const auto guessed = autoviz::example::GuessAutolinkWorkRoot();
    if (!guessed.empty()) {
      ::setenv("AUTOLINK_PATH", guessed.c_str(), 0);
    }
  }
  autolink::Init(argv[0]);
  autoviz::example::Run();
  return 0;
}
