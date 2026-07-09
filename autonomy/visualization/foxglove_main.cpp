/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "autolink/init.hpp"
#include "autonomy/visualization/visualization_server.hpp"

DEFINE_string(foxglove_host, "0.0.0.0",
              "Host address exposed by the Foxglove bridge.");
DEFINE_int32(foxglove_port, 8765,
             "Port exposed by the Foxglove bridge.");
DEFINE_string(
    channel_allowlist, "",
    "Comma-separated autolink channel allowlist. Empty means all channels.");
DEFINE_string(
    message_type_allowlist, "",
    "Comma-separated message type allowlist. Each item can be an exact type "
    "or a prefix. Empty means forward all registry-supported message types.");
DEFINE_int32(discovery_poll_interval_ms, 1000,
             "Polling interval for discovering new autolink channels.");
DEFINE_int32(time_broadcast_interval_ms, 100,
             "Interval for Foxglove live time sync (3D panel needs this).");

namespace {

std::vector<std::string> SplitCsv(const std::string& csv) {
  std::vector<std::string> values;
  std::stringstream stream(csv);
  std::string item;
  while (std::getline(stream, item, ',')) {
    if (!item.empty()) {
      values.push_back(item);
    }
  }
  return values;
}

}  // namespace

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  if (!autolink::Init(argv[0])) {
    LOG(ERROR) << "Failed to initialize autolink runtime.";
    return 1;
  }

  autonomy::visualization::BridgeRuntimeOptions options;
  options.foxglove.host = FLAGS_foxglove_host;
  options.foxglove.port = static_cast<uint16_t>(FLAGS_foxglove_port);
  options.poll_interval_ms = FLAGS_discovery_poll_interval_ms;
  options.time_broadcast_interval_ms = FLAGS_time_broadcast_interval_ms;
  options.channel_allowlist = SplitCsv(FLAGS_channel_allowlist);
  options.message_type_allowlist = SplitCsv(FLAGS_message_type_allowlist);

  autonomy::visualization::VisualizationServer app(std::move(options));
  if (!app.Start()) {
    return 1;
  }

  app.Spin();
  autolink::Clear();
  return 0;
}
