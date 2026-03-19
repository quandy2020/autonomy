#include "autonomy/autoviz/core/config/config.hpp"

#include <yaml-cpp/yaml.h>

namespace autoviz {
namespace config {

namespace {

FoxgloveConfig ParseFoxglove(const YAML::Node& node) {
  FoxgloveConfig cfg;
  if (!node || !node.IsMap()) {
    return cfg;
  }

  if (auto v = node["host"]) {
    cfg.host = v.as<std::string>(cfg.host);
  }
  if (auto v = node["port"]) {
    cfg.port = v.as<int>(cfg.port);
  }
  if (auto v = node["session_id"]) {
    cfg.session_id = v.as<std::string>(cfg.session_id);
  }
  if (auto v = node["capabilities"]) {
    cfg.capabilities = v.as<std::vector<std::string>>(cfg.capabilities);
  }
  if (auto v = node["supported_encodings"]) {
    cfg.supported_encodings =
        v.as<std::vector<std::string>>(cfg.supported_encodings);
  }
  if (auto v = node["send_buffer_limit_bytes"]) {
    cfg.send_buffer_limit_bytes =
        v.as<std::size_t>(cfg.send_buffer_limit_bytes);
  }
  if (auto v = node["use_compression"]) {
    cfg.use_compression = v.as<bool>(cfg.use_compression);
  }
  if (auto v = node["use_ros2_type_name"]) {
    cfg.use_ros2_type_name = v.as<bool>(cfg.use_ros2_type_name);
  }
  if (auto tls = node["tls"]) {
    if (auto v = tls["enabled"]) {
      cfg.tls.enabled = v.as<bool>(cfg.tls.enabled);
    }
    if (auto v = tls["cert_file"]) {
      cfg.tls.cert_file = v.as<std::string>(cfg.tls.cert_file);
    }
    if (auto v = tls["key_file"]) {
      cfg.tls.key_file = v.as<std::string>(cfg.tls.key_file);
    }
  }
  return cfg;
}

AutolinkConfig ParseAutolink(const YAML::Node& node) {
  AutolinkConfig cfg;
  if (!node || !node.IsMap()) {
    return cfg;
  }

  if (auto v = node["topic_whitelist"]) {
    cfg.topic_whitelist = v.as<std::vector<std::string>>(cfg.topic_whitelist);
  }
  if (auto v = node["min_update_period_ms"]) {
    cfg.min_update_period_ms = v.as<double>(cfg.min_update_period_ms);
  }
  if (auto v = node["max_update_period_ms"]) {
    cfg.max_update_period_ms = v.as<double>(cfg.max_update_period_ms);
  }
  return cfg;
}

McapConfig ParseMcap(const YAML::Node& node) {
  McapConfig cfg;
  if (!node || !node.IsMap()) {
    return cfg;
  }
  if (auto v = node["enabled"]) {
    cfg.enabled = v.as<bool>(cfg.enabled);
  }
  if (auto v = node["output_path"]) {
    cfg.output_path = v.as<std::string>(cfg.output_path);
  }
  if (auto v = node["flush_on_write"]) {
    cfg.flush_on_write = v.as<bool>(cfg.flush_on_write);
  }
  return cfg;
}

}  // namespace

Config LoadConfig(const std::string& path) {
  Config cfg;
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception&) {
    // Fall back to defaults if loading fails.
    return cfg;
  }

  cfg.foxglove = ParseFoxglove(root["foxglove"]);
  cfg.autolink = ParseAutolink(root["autolink"]);
  cfg.mcap = ParseMcap(root["mcap"]);
  return cfg;
}

}  // namespace config
}  // namespace autoviz

