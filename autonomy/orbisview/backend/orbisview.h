/*
 * Copyright 2026 The Openbot Authors
 *
 * OrbisView application server (Dreamview dreamview.h counterpart):
 * CivetWeb HTTP+WS, mock/autolink sources, throttle, stats, recorder, plugins.
 */

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "autonomy/orbisview/backend/adapters/mock/mock_source.h"
#include "autonomy/orbisview/backend/common/handlers/websocket_handler.h"
#include "autonomy/orbisview/backend/common/map_service/map_service.h"
#include "autonomy/orbisview/backend/common/plugins/plugin_host.h"
#include "autonomy/orbisview/backend/common/plugins/registry.h"
#include "autonomy/orbisview/backend/common/streaming/channel_stats.h"
#include "autonomy/orbisview/backend/common/streaming/throttle_queue.h"
#include "autonomy/orbisview/backend/hmi/hmi.h"
#include "autonomy/orbisview/backend/perception_camera_updater/perception_camera_updater.h"
#include "autonomy/orbisview/backend/point_cloud/point_cloud_updater.h"
#include "autonomy/orbisview/backend/record_player/envelope_recorder.h"
#include "autonomy/orbisview/backend/simulation_world/simulation_world_updater.h"
#include "autonomy/orbisview/backend/teleop/teleop.h"
#include "autonomy/orbisview/backend/common/stream_envelope.h"
#include "autonomy/orbisview/backend/common/subscription.h"

#if defined(ORBISVIEW_WITH_AUTOLINK)
#include "autolink/node/node.hpp"
#include "autolink/service_discovery/topology_manager.hpp"
#endif

class CivetServer;

namespace autonomy {
namespace orbisview {
namespace backend {

struct ServerOptions {
  std::string host{"127.0.0.1"};
  uint16_t port{8766};
  bool enable_mock{true};
  bool enable_autolink{false};
  std::string document_root;
  std::string plugin_dir;
  std::string cmd_vel_channel{"/cmd_vel"};
  /** Optional conf/hmi_modes directory for extra mode JSON files. */
  std::string hmi_modes_dir;
};

/**
 * @class Orbisview
 * @brief Top-level server (mirrors apollo::dreamview::Dreamview lifecycle).
 */
class Orbisview {
 public:
  Orbisview();
  ~Orbisview();

  bool Init(const ServerOptions& options);
  bool Start();
  void Stop();

  size_t ClientCount() const {
    return websocket_ ? websocket_->ClientCount() : 0;
  }
  uint64_t DroppedFrames() const { return outbound_.DroppedFrames(); }

 private:
  void OnClientMessage(int client_id, const std::string& text);
  void OnConnect(int client_id);
  void OnDisconnect(int client_id);
  void HandleListChannels(int client_id);
  void HandleSubscribe(int client_id, const std::string& channel, double max_hz);
  void HandleUnsubscribe(int client_id, const std::string& channel);
  void HandleStatus(int client_id);
  void EmitEnvelope(core::StreamEnvelope env);
  void IngestWorld(const core::StreamEnvelope& env);
  void PumpLoop();
  std::vector<core::ChannelInfo> CollectChannels() const;
  bool ShouldForward(int client_id, const std::string& channel,
                     core::SubscriptionState* state);
  void BroadcastChannels();

  ServerOptions options_;
  bool initialized_{false};

  std::unique_ptr<CivetServer> server_;
  std::unique_ptr<WebSocketHandler> websocket_;
  std::unique_ptr<SimulationWorldUpdater> sim_world_updater_;
  std::unique_ptr<MapService> map_service_;
  std::unique_ptr<core::Hmi> hmi_;
  std::unique_ptr<PointCloudUpdater> point_cloud_updater_;
  std::unique_ptr<PerceptionCameraUpdater> perception_camera_updater_;
  std::unique_ptr<TeleopService> teleop_;
  std::unique_ptr<plugins::PluginHost> plugin_host_;

  ThrottleQueue outbound_;
  ChannelStatsTracker stats_;
  EnvelopeRecorder recorder_;
  plugins::PluginRegistry plugins_;
  adapters::MockSource mock_;

  std::atomic<bool> running_{false};
  std::thread pump_thread_;
  std::chrono::steady_clock::time_point last_topo_refresh_{};

  mutable std::mutex mutex_;
  std::unordered_map<int, std::unordered_map<std::string, core::SubscriptionState>>
      subs_;

#if defined(ORBISVIEW_WITH_AUTOLINK)
  std::shared_ptr<autolink::Node> node_;
  std::unordered_map<std::string, std::shared_ptr<void>> readers_;
  void EnsureAutolinkSubscribe(const std::string& channel);
  void RefreshAutolinkChannels();
  bool RefreshAutolinkChannelsChanged();
  void PublishCmdVelAutolink(double vx, double wz);
  std::vector<core::ChannelInfo> autolink_channels_;
  std::string autolink_fingerprint_;
#if defined(ORBISVIEW_WITH_AUTOMSGS)
  std::shared_ptr<void> cmd_vel_writer_;
#endif
#endif
};

}  // namespace backend
}  // namespace orbisview
}  // namespace autonomy
