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

#include <yaml-cpp/yaml.h>

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

static std::string GuessAutovizConfigPath() {
  namespace fs = std::filesystem;
  const fs::path cwd = fs::current_path();
  fs::path base = cwd;
  for (int depth = 0; depth < 10; ++depth) {
    const fs::path cfg = base / "src" / "autonomy" / "autoviz" / "config" / "autoviz.yaml";
    std::error_code ec;
    if (fs::exists(cfg, ec) && !ec) {
      return cfg.string();
    }
    if (!base.has_parent_path()) break;
    base = base.parent_path();
  }
  return "";
}

static void LoadOptionsFromYaml(const std::string& path, AutolinkBridge::Options* options) {
  if (path.empty() || options == nullptr) {
    return;
  }
  try {
    const YAML::Node root = YAML::LoadFile(path);
    const YAML::Node fox = root["foxglove"];
    if (fox) {
      if (fox["host"]) options->host = fox["host"].as<std::string>();
      if (fox["port"]) options->port = fox["port"].as<uint16_t>();
      if (fox["session_id"]) options->session_id = fox["session_id"].as<std::string>();
      if (fox["send_buffer_limit_bytes"]) {
        options->send_buffer_limit_bytes = fox["send_buffer_limit_bytes"].as<std::size_t>();
      }
      if (fox["use_compression"]) options->use_compression = fox["use_compression"].as<bool>();

      if (fox["capabilities"] && fox["capabilities"].IsSequence()) {
        options->capabilities.clear();
        for (const auto& item : fox["capabilities"]) {
          options->capabilities.emplace_back(item.as<std::string>());
        }
      }
      if (fox["supported_encodings"] && fox["supported_encodings"].IsSequence()) {
        options->supported_encodings.clear();
        for (const auto& item : fox["supported_encodings"]) {
          options->supported_encodings.emplace_back(item.as<std::string>());
        }
      }
      const YAML::Node tls = fox["tls"];
      if (tls) {
        if (tls["enabled"]) options->use_tls = tls["enabled"].as<bool>();
        if (tls["cert_file"]) options->cert_file = tls["cert_file"].as<std::string>();
        if (tls["key_file"]) options->key_file = tls["key_file"].as<std::string>();
      }
    }

    const YAML::Node al = root["autolink"];
    if (al) {
      if (al["min_update_period_ms"]) {
        options->min_update_period_ms = al["min_update_period_ms"].as<double>();
      }
      if (al["max_update_period_ms"]) {
        options->max_update_period_ms = al["max_update_period_ms"].as<double>();
      }
      if (al["topic_whitelist"] && al["topic_whitelist"].IsSequence()) {
        options->topic_whitelist.clear();
        for (const auto& item : al["topic_whitelist"]) {
          options->topic_whitelist.emplace_back(item.as<std::string>());
        }
      }
    }
    AINFO << "autoviz: loaded config " << path;
  } catch (const std::exception& e) {
    AWARN << "autoviz: failed to load config " << path << ", reason: " << e.what();
  }
}

namespace {

void Run() {
  signal(SIGINT, [](int) { exit(0); });
  signal(SIGTERM, [](int) { exit(0); });

  AutolinkBridge::Options options;
  const char* env_cfg = std::getenv("AUTOVIZ_CONFIG");
  const std::string cfg_path = (env_cfg && env_cfg[0] != '\0') ? env_cfg : GuessAutovizConfigPath();
  LoadOptionsFromYaml(cfg_path, &options);

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
