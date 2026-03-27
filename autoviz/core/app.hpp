// autoviz：Foxglove WebSocket + autolink 订阅/转发 + 可选 MCAP。
#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>

#include "autonomy/autoviz/core/settings.hpp"

namespace autoviz {

namespace server {
class FoxgloveServer;
}  // namespace server

namespace autolink {
class AutolinkBridge;
}  // namespace autolink

namespace recorder {
class Recorder;
}  // namespace recorder

class App {
 public:
  App();
  ~App();

  bool Initialize(const std::string& config_path);
  int Run();
  void Shutdown();

 private:
  bool initialized_{false};
  std::atomic_bool shutdown_requested_{false};
  std::mutex shutdown_mutex_;
  std::condition_variable shutdown_cv_;

  std::shared_ptr<config::Config> config_;
  std::unique_ptr<server::FoxgloveServer> foxglove_server_;
  std::unique_ptr<recorder::Recorder> mcap_recorder_;
  std::unique_ptr<autolink::AutolinkBridge> autolink_bridge_;
};

}  // namespace autoviz
