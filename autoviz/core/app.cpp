#include "autonomy/autoviz/core/app.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "autonomy/autoviz/core/config/config.hpp"
#include "autonomy/autoviz/core/server/foxglove_server.hpp"
#include "autonomy/autoviz/core/bridge/autolink_bridge.hpp"
#include "autonomy/autoviz/core/mcap/recorder.hpp"
#include "autonomy/autoviz/core/transport/auto_discovery.hpp"

namespace autoviz {

App::App() = default;

App::~App() = default;

bool App::Init(const std::string& config_path) {
  config_ = std::make_shared<config::Config>(config::LoadConfig(config_path));

  foxglove_server_ =
      std::make_unique<server::FoxgloveServer>(config_->foxglove);
  autolink_bridge_ =
      std::make_unique<bridge::AutolinkBridge>(config_->autolink, *foxglove_server_);
  mcap_recorder_ =
      std::make_unique<mcap::Recorder>(config_->mcap);
  auto_discovery_ =
      std::make_unique<transport::AutoDiscovery>(config_->autolink, *autolink_bridge_,
                                                 mcap_recorder_.get());

  foxglove_server_->Start();
  autolink_bridge_->Start();
  mcap_recorder_->Start();
  auto_discovery_->Start();

  initialized_ = true;
  return true;
}

int App::Run() {
  if (!initialized_) {
    return 1;
  }

  // Simple polling loop for now; can be replaced with event-driven logic.
  while (!shutdown_requested_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  auto_discovery_->Stop();
  mcap_recorder_->Stop();
  autolink_bridge_->Stop();
  foxglove_server_->Stop();

  return 0;
}

void App::RequestShutdown() {
  shutdown_requested_ = true;
}

}  // namespace autoviz

