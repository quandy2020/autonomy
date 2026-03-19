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
#include <memory>
#include <thread>

#include "autolink/autolink.hpp"
#include "autolink/common/log.hpp"
#include "autolink_bridge.hpp"

namespace autoviz {

static std::string GuessAutolinkWorkRoot() {
  namespace fs = std::filesystem;
  const fs::path cwd = fs::current_path();
  const fs::path conf_rel = fs::path("conf") / "autolink.pb.conf";
  fs::path base = cwd;
  for (int depth = 0; depth < 10; ++depth) {
    const fs::path conf =
        base / "src" / "autonomy" / "autolink" / "autolink" / conf_rel;
    std::error_code ec;
    if (fs::exists(conf, ec) && !ec) {
      return conf.parent_path().parent_path().string();
    }
    if (!base.has_parent_path()) break;
    base = base.parent_path();
  }
  return "";
}

namespace {

void Run() {
  signal(SIGINT, [](int) { exit(0); });
  signal(SIGTERM, [](int) { exit(0); });

  AutolinkBridge::Options options;
  options.host = "0.0.0.0";
  options.port = 8765;

  auto bridge = std::make_unique<AutolinkBridge>(options);
  if (!bridge->Start()) {
    AERROR << "autoviz: Failed to start bridge";
    return;
  }
  AINFO << "autoviz: Foxglove bridge running (autolink data forwarding)";

  while (true) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

}  // namespace
}  // namespace autoviz

int main(int argc, char** argv) {
  if (std::getenv("AUTOLINK_PATH") == nullptr) {
    const auto guessed = autoviz::GuessAutolinkWorkRoot();
    if (!guessed.empty()) {
      ::setenv("AUTOLINK_PATH", guessed.c_str(), 0);
    }
  }

  autolink::Init(argv[0]);
  autoviz::Run();
  return 0;
}
