#include "autonomy/autoviz/core/app.hpp"

#include <cstddef>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>

#include "autonomy/autoviz/core/autolink/autolink_bridge.hpp"
#include "autonomy/autoviz/core/foxglove/foxglove_server.hpp"
#include "autonomy/autoviz/core/recorder/recorder.hpp"

#include "autolink/common/file.hpp"
#include "autoviz_conf.pb.h"

namespace autoviz {
namespace {

config::Config FromAutovizConfProto(const autoviz::conf::AutovizConf& pb) {
  config::Config c;

  if (pb.has_foxglove()) {
    const auto& f = pb.foxglove();
    c.foxglove.host = f.host();
    c.foxglove.port = f.port();
    c.foxglove.session_id = f.session_id();
    if (f.capabilities_size() > 0) {
      c.foxglove.capabilities.assign(f.capabilities().begin(), f.capabilities().end());
    }
    if (f.supported_encodings_size() > 0) {
      c.foxglove.supported_encodings.assign(f.supported_encodings().begin(),
                                            f.supported_encodings().end());
    }
    c.foxglove.send_buffer_limit_bytes =
        static_cast<std::size_t>(f.send_buffer_limit_bytes());
    c.foxglove.use_compression = f.use_compression();
    c.foxglove.use_ros2_type_name = f.use_ros2_type_name();
    if (f.has_tls()) {
      c.foxglove.tls.enabled = f.tls().enabled();
      c.foxglove.tls.cert_file = f.tls().cert_file();
      c.foxglove.tls.key_file = f.tls().key_file();
    }
  }

  if (pb.has_autolink()) {
    const auto& a = pb.autolink();
    if (a.topic_whitelist_size() > 0) {
      c.autolink.topic_whitelist.assign(a.topic_whitelist().begin(), a.topic_whitelist().end());
    }
    c.autolink.min_update_period_ms = a.min_update_period_ms();
    c.autolink.max_update_period_ms = a.max_update_period_ms();
  }

  if (pb.has_mcap()) {
    const auto& m = pb.mcap();
    c.mcap.enabled = m.enabled();
    c.mcap.output_path = m.output_path();
    c.mcap.flush_on_write = m.flush_on_write();
  }

  return c;
}

config::Config LoadAutovizPbConf(const std::string& path) {
  autoviz::conf::AutovizConf pb;
  if (!::autolink::common::LoadConfig(path, &pb)) {
    return config::Config{};
  }
  return FromAutovizConfProto(pb);
}



}  // namespace

App::App() = default;
App::~App() = default;

bool App::Initialize(const std::string& config_path) {
  config_ = std::make_shared<config::Config>(LoadAutovizPbConf(config_path));
  shutdown_requested_.store(false);

  foxglove_server_ = std::make_unique<server::FoxgloveServer>(config_->foxglove);
  mcap_recorder_ = std::make_unique<recorder::Recorder>(config_->mcap);
  autolink_bridge_ = std::make_unique<autolink::AutolinkBridge>(
      config_->autolink, *foxglove_server_, mcap_recorder_.get());

  foxglove_server_->Start();
  autolink_bridge_->Start();
  mcap_recorder_->Start();

  std::cout << "autoviz: foxglove ws + autolink->foxglove"
            << (mcap_recorder_->IsEnabled() ? " + mcap" : "") << std::endl;
  initialized_ = true;
  return true;
}

int App::Run() {
  if (!initialized_) {
    return 1;
  }

  {
    std::unique_lock<std::mutex> lock(shutdown_mutex_);
    shutdown_cv_.wait(lock, [this] { return shutdown_requested_.load(); });
  }

  mcap_recorder_->Stop();
  autolink_bridge_->Stop();
  foxglove_server_->Stop();

  return 0;
}

void App::Shutdown() {
  shutdown_requested_.store(true);
  shutdown_cv_.notify_all();
}

}  // namespace autoviz
