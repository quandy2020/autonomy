// Configuration structures and loading for autoviz.
#pragma once

#include <string>
#include <vector>

namespace autoviz {
namespace config {

struct FoxgloveTlsConfig {
  bool enabled{false};
  std::string cert_file;
  std::string key_file;
};

struct FoxgloveConfig {
  std::string host{"0.0.0.0"};
  int port{8765};
  std::string session_id{"autoviz"};
  std::vector<std::string> capabilities;
  std::vector<std::string> supported_encodings;
  std::size_t send_buffer_limit_bytes{10 * 1000 * 1000};
  bool use_compression{false};
  bool use_ros2_type_name{true};
  FoxgloveTlsConfig tls;
};

struct AutolinkConfig {
  std::vector<std::string> topic_whitelist;
  double min_update_period_ms{100.0};
  double max_update_period_ms{5000.0};
};

struct McapConfig {
  bool enabled{true};
  std::string output_path{"autoviz.mcap"};
  bool flush_on_write{false};
};

struct Config {
  FoxgloveConfig foxglove;
  AutolinkConfig autolink;
  McapConfig mcap;
};

// Load configuration from YAML file at the given path.
// For now, missing fields fall back to the defaults defined above.
Config LoadConfig(const std::string& path);

}  // namespace config
}  // namespace autoviz

