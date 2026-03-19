// High-level application wiring for autoviz.
#pragma once

#include <memory>
#include <string>

namespace autoviz {

namespace config {
struct Config;
}  // namespace config

namespace server {
class FoxgloveServer;
}  // namespace server

namespace bridge {
class AutolinkBridge;
}  // namespace bridge

namespace transport {
class AutoDiscovery;
}  // namespace transport

namespace mcap {
class Recorder;
}  // namespace mcap

// Orchestrates configuration, foxglove server, autolink bridge and discovery.
class App {
 public:
  App();
  ~App();

  // Initialize from config file path. Returns false on failure.
  bool Init(const std::string& config_path);

  // Blocking run loop. Returns when shutdown is requested or on error.
  int Run();

  // Trigger a graceful shutdown.
  void RequestShutdown();

 private:
  bool initialized_{false};
  bool shutdown_requested_{false};

  std::shared_ptr<config::Config> config_;
  std::unique_ptr<server::FoxgloveServer> foxglove_server_;
  std::unique_ptr<bridge::AutolinkBridge> autolink_bridge_;
  std::unique_ptr<mcap::Recorder> mcap_recorder_;
  std::unique_ptr<transport::AutoDiscovery> auto_discovery_;
};

}  // namespace autoviz

