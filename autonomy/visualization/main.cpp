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
#include <cstdlib>
#include <filesystem>
#include <thread>

#include "autolink/autolink.hpp"
#include "autonomy/common/gflags.hpp"
#include "autonomy/common/version.hpp"
#include "autonomy/visualization/bridge/foxglove_bridge.hpp"
#include "autonomy/visualization/transport/auto_discovery.hpp"

namespace autonomy {
namespace visualization {
std::string GuessAutolinkWorkRoot() {
  namespace fs = std::filesystem;
  const fs::path cwd = fs::current_path();
  const fs::path conf_rel = fs::path("conf") / "autolink.pb.conf";

  // 从当前目录开始向上回溯若干层，查找常见源码布局：
  // <repo_root>/src/autonomy/autolink/autolink/conf/autolink.pb.conf
  fs::path base = cwd;
  for (int depth = 0; depth < 10; ++depth) {
    const fs::path conf = base / "src" / "autonomy" / "autolink" / "autolink" / conf_rel;
    std::error_code ec;
    if (fs::exists(conf, ec) && !ec) {
      return conf.parent_path().parent_path().string();  // .../autolink/autolink
    }
    if (!base.has_parent_path()) {
      break;
    }
    base = base.parent_path();
  }
  return "";
}

namespace {

void Run() {
  // register signal handler with lambda
  signal(SIGINT, [](int) { exit(0); });
  signal(SIGTERM, [](int) { exit(0); });

  // configure FoxgloveBridge
  FoxgloveBridge::Options options;
  options.host = "0.0.0.0";
  options.port = 8765;

  // start FoxgloveBridge
  auto bridge = std::make_shared<FoxgloveBridge>(options);
  if (!bridge->Start()) {
    AERROR << "Failed to start visualization server";
    return;
  } else {
    AINFO << "Visualization server started successfully";
  }

  // wait for signal
  while (true) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

}  // namespace
}  // namespace visualization
}  // namespace autonomy

int main(int argc, char** argv) {
  if (std::getenv("AUTOLINK_PATH") == nullptr) {
    const auto guessed = autonomy::visualization::GuessAutolinkWorkRoot();
    if (!guessed.empty()) {
      ::setenv("AUTOLINK_PATH", guessed.c_str(), /*overwrite=*/0);
    }
  }
  autolink::Init(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  if (autonomy::common::FLAGS_verbose) {
    autonomy::common::ShowVersion();
    return 0;
  }
  autonomy::visualization::Run();
  google::ShutDownCommandLineFlags();
  return 0;
}
