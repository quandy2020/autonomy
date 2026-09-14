/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/orbisview/backend/orbisview.hpp"

#include <glog/logging.h>

#include <chrono>
#include <fstream>
#include <sstream>
#include <unordered_set>

#if !defined(_WIN32)
#include <dirent.h>
#endif

#include "CivetServer.h"

#include "autonomy/orbisview/backend/common/render_schemas.hpp"

#if defined(ORBISVIEW_WITH_AUTOLINK)
#include "autolink/autolink.hpp"
#include "autolink/message/raw_message.hpp"
#include "autolink/node/writer.hpp"
#if defined(ORBISVIEW_WITH_AUTOMSGS)
#include "autonomy/orbisview/backend/adapters/automsgs/automsgs_converter.hpp"
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#endif
#endif

#include <utility>
#include <vector>
namespace autonomy {
namespace orbisview {
namespace backend {
namespace {

std::string ExtractJsonString(const std::string& text, const std::string& key) {
  const std::string needle = "\"" + key + "\"";
  auto pos = text.find(needle);
  if (pos == std::string::npos) return {};
  pos = text.find(':', pos + needle.size());
  if (pos == std::string::npos) return {};
  ++pos;
  while (pos < text.size() && (text[pos] == ' ' || text[pos] == '\t')) ++pos;
  if (pos >= text.size() || text[pos] != '"') return {};
  ++pos;
  std::string out;
  while (pos < text.size() && text[pos] != '"') {
    if (text[pos] == '\\' && pos + 1 < text.size()) {
      out.push_back(text[pos + 1]);
      pos += 2;
      continue;
    }
    out.push_back(text[pos++]);
  }
  return out;
}

std::string ExtractJsonOp(const std::string& text) {
  return ExtractJsonString(text, "op");
}

double ExtractJsonNumber(const std::string& text, const std::string& key,
                         double fallback) {
  const std::string needle = "\"" + key + "\"";
  auto pos = text.find(needle);
  if (pos == std::string::npos) return fallback;
  pos = text.find(':', pos + needle.size());
  if (pos == std::string::npos) return fallback;
  try {
    return std::stod(text.substr(pos + 1));
  } catch (...) {
    return fallback;
  }
}

bool ExtractJsonBool(const std::string& text, const std::string& key,
                     bool fallback) {
  const std::string needle = "\"" + key + "\"";
  auto pos = text.find(needle);
  if (pos == std::string::npos) return fallback;
  pos = text.find(':', pos + needle.size());
  if (pos == std::string::npos) return fallback;
  ++pos;
  while (pos < text.size() && (text[pos] == ' ' || text[pos] == '\t')) ++pos;
  if (text.compare(pos, 4, "true") == 0) return true;
  if (text.compare(pos, 5, "false") == 0) return false;
  return fallback;
}

std::unordered_set<std::string> ExtractJsonStringArray(
    const std::string& text, const std::string& key) {
  std::unordered_set<std::string> out;
  const std::string needle = "\"" + key + "\"";
  auto pos = text.find(needle);
  if (pos == std::string::npos) return out;
  pos = text.find('[', pos + needle.size());
  if (pos == std::string::npos) return out;
  ++pos;
  while (pos < text.size() && text[pos] != ']') {
    while (pos < text.size() &&
           (text[pos] == ' ' || text[pos] == ',' || text[pos] == '\n' ||
            text[pos] == '\t')) {
      ++pos;
    }
    if (pos >= text.size() || text[pos] == ']') break;
    if (text[pos] != '"') break;
    ++pos;
    std::string item;
    while (pos < text.size() && text[pos] != '"') {
      if (text[pos] == '\\' && pos + 1 < text.size()) {
        item.push_back(text[pos + 1]);
        pos += 2;
        continue;
      }
      item.push_back(text[pos++]);
    }
    if (pos < text.size() && text[pos] == '"') ++pos;
    out.insert(item);
  }
  return out;
}

std::vector<adapters::MockSource::RoutePoint> ExtractWaypoints(
    const std::string& text) {
  std::vector<adapters::MockSource::RoutePoint> out;
  const std::string needle = "\"waypoints\"";
  auto pos = text.find(needle);
  if (pos == std::string::npos) return out;
  pos = text.find('[', pos + needle.size());
  if (pos == std::string::npos) return out;
  ++pos;
  while (pos < text.size() && text[pos] != ']') {
    auto obj = text.find('{', pos);
    if (obj == std::string::npos || obj > text.find(']', pos)) break;
    auto end = text.find('}', obj);
    if (end == std::string::npos) break;
    const std::string obj_text = text.substr(obj, end - obj + 1);
    out.push_back({ExtractJsonNumber(obj_text, "x", 0.0),
                   ExtractJsonNumber(obj_text, "y", 0.0),
                   ExtractJsonNumber(obj_text, "yaw", 0.0)});
    pos = end + 1;
  }
  return out;
}

}  // namespace

Orbisview::Orbisview() = default;

Orbisview::~Orbisview() { Stop(); }

bool Orbisview::Init(const ServerOptions& options) {
  if (initialized_) return true;
  options_ = options;
  websocket_ = std::make_unique<WebSocketHandler>();
  sim_world_updater_ = std::make_unique<SimulationWorldUpdater>();
  hmi_ = std::make_unique<core::HmiWorker>();
  teleop_ = std::make_unique<TeleopService>();
  plugins_.RegisterBuiltins();
  plugin_host_ = std::make_unique<plugins::PluginHost>(&plugins_);
  if (!options_.hmi_modes_dir.empty()) {
    hmi_->LoadModesDir(options_.hmi_modes_dir);
  }
  if (!options_.plugin_dir.empty()) {
    const int n = plugin_host_->ScanAndLoad(options_.plugin_dir);
    LOG(INFO) << "OrbisView plugins scanned " << options_.plugin_dir
              << " loaded=" << n;
  }
  websocket_->SetConnectHandler([this](int id) { OnConnect(id); });
  websocket_->SetDisconnectHandler([this](int id) { OnDisconnect(id); });
  websocket_->SetMessageHandler(
      [this](int id, const std::string& text) { OnClientMessage(id, text); });
#if defined(ORBISVIEW_WITH_AUTOLINK)
  teleop_->SetPublisher([this](double vx, double wz) {
    if (options_.enable_autolink) PublishCmdVelAutolink(vx, wz);
  });
#endif
  initialized_ = true;
  return true;
}

bool Orbisview::Start() {
  if (!initialized_ && !Init(options_)) return false;
  if (running_.exchange(true)) return true;
  const std::string port = std::to_string(options_.port);
  const std::string listen =
      (options_.host == "0.0.0.0" || options_.host.empty())
          ? port
          : (options_.host + ":" + port);
  std::vector<std::string> opt_storage = {
      "listening_ports", listen,
      "num_threads", "4",
      "enable_websocket_ping_pong", "yes",
  };
  if (!options_.document_root.empty()) {
    opt_storage.push_back("document_root");
    opt_storage.push_back(options_.document_root);
  }
  std::vector<const char*> server_opts;
  server_opts.reserve(opt_storage.size() + 1);
  for (const auto& s : opt_storage) {
    server_opts.push_back(s.c_str());
  }
  server_opts.push_back(nullptr);

  try {
    server_ = std::make_unique<CivetServer>(server_opts.data());
  } catch (const CivetException& e) {
    LOG(ERROR) << "CivetServer start failed: " << e.what();
    running_ = false;
    return false;
  }
  server_->addWebSocketHandler("/ws", *websocket_);
  LOG(INFO) << "OrbisView CivetWeb on http://" << options_.host << ':'
            << options_.port << "  ws path=/ws"
            << (options_.document_root.empty()
                    ? ""
                    : ("  static=" + options_.document_root));

  if (options_.enable_mock) {
    mock_.SetEmit([this](core::StreamEnvelope env) { EmitEnvelope(std::move(env)); });
    mock_.Start();
  }

#if defined(ORBISVIEW_WITH_AUTOLINK)
  if (options_.enable_autolink) {
    node_ = autolink::CreateNode("orbisview");
    RefreshAutolinkChannels();
  }
#else
  if (options_.enable_autolink) {
    LOG(WARNING) << "OrbisView built without Autolink; ignoring --autolink";
  }
#endif

  pump_thread_ = std::thread([this] { PumpLoop(); });
  return true;
}

void Orbisview::Stop() {
  if (!running_.exchange(false)) return;
  mock_.Stop();
  recorder_.StopPlayback();
  recorder_.StopRecording();
  if (pump_thread_.joinable()) pump_thread_.join();
  plugin_host_.reset();
  server_.reset();
  outbound_.Clear();
  std::lock_guard<std::mutex> lock(mutex_);
  subs_.clear();
#if defined(ORBISVIEW_WITH_AUTOLINK)
  readers_.clear();
  node_.reset();
#endif
}

void Orbisview::OnConnect(int client_id) {
  HandleListChannels(client_id);
  websocket_->SendText(client_id, plugins_.ToJson());
}

void Orbisview::OnDisconnect(int client_id) {
  std::lock_guard<std::mutex> lock(mutex_);
  subs_.erase(client_id);
}

void Orbisview::OnClientMessage(int client_id, const std::string& text) {
  const auto op = ExtractJsonOp(text);
  if (op == "list_channels") {
    HandleListChannels(client_id);
  } else if (op == "subscribe") {
    HandleSubscribe(client_id, ExtractJsonString(text, "channel"),
                    ExtractJsonNumber(text, "max_hz", 0.0));
  } else if (op == "unsubscribe") {
    HandleUnsubscribe(client_id, ExtractJsonString(text, "channel"));
  } else if (op == "status") {
    HandleStatus(client_id);
  } else if (op == "channel_stats") {
    websocket_->SendText(client_id, stats_.ToJson());
  } else if (op == "list_plugins") {
    websocket_->SendText(client_id, plugins_.ToJson());
  } else if (op == "plugins_scan") {
    const auto dir = ExtractJsonString(text, "path");
    const std::string scan_dir =
        dir.empty() ? options_.plugin_dir : dir;
    if (scan_dir.empty() || !plugin_host_) {
      websocket_->SendText(client_id,
                   "{\"op\":\"error\",\"message\":\"plugins_scan needs path\"}");
    } else {
      plugin_host_->ScanAndLoad(scan_dir);
      websocket_->SendText(client_id, plugin_host_->StatusJson());
      websocket_->SendText(client_id, plugins_.ToJson());
    }
  } else if (op == "plugins_load") {
    const auto path = ExtractJsonString(text, "path");
    if (path.empty() || !plugin_host_) {
      websocket_->SendText(client_id,
                   "{\"op\":\"error\",\"message\":\"plugins_load needs path\"}");
    } else {
      const bool ok = plugin_host_->Load(path);
      websocket_->SendText(client_id, ok ? plugin_host_->StatusJson()
                                 : "{\"op\":\"error\",\"message\":\"plugins_load failed\"}");
      if (ok) websocket_->SendText(client_id, plugins_.ToJson());
    }
  } else if (op == "plugins_unload") {
    const auto id = ExtractJsonString(text, "id");
    if (id.empty() || !plugin_host_) {
      websocket_->SendText(client_id,
                   "{\"op\":\"error\",\"message\":\"plugins_unload needs id\"}");
    } else {
      const bool ok = plugin_host_->Unload(id);
      websocket_->SendText(client_id, ok ? plugin_host_->StatusJson()
                                 : "{\"op\":\"error\",\"message\":\"plugins_unload failed\"}");
      if (ok) websocket_->SendText(client_id, plugins_.ToJson());
    }
  } else if (op == "plugins_reload") {
    const auto id = ExtractJsonString(text, "id");
    if (id.empty() || !plugin_host_) {
      websocket_->SendText(client_id,
                   "{\"op\":\"error\",\"message\":\"plugins_reload needs id\"}");
    } else {
      const bool ok = plugin_host_->Reload(id);
      websocket_->SendText(client_id, ok ? plugin_host_->StatusJson()
                                 : "{\"op\":\"error\",\"message\":\"plugins_reload failed\"}");
      if (ok) websocket_->SendText(client_id, plugins_.ToJson());
    }
  } else if (op == "plugin_host_status") {
    websocket_->SendText(client_id, plugin_host_ ? plugin_host_->StatusJson()
                                         : "{\"op\":\"plugin_host\",\"loaded\":[],\"failures\":[]}");
  } else if (op == "recorder_status") {
    websocket_->SendText(client_id, recorder_.StatusJson());
  } else if (op == "record_start") {
    RecordOptions ro;
    ro.path = ExtractJsonString(text, "path");
    if (ro.path.empty()) ro.path = "/tmp/orbisview_record.jsonl";
    ro.channels = ExtractJsonStringArray(text, "channels");
    const bool ok = recorder_.StartRecording(ro);
    websocket_->SendText(client_id, ok ? recorder_.StatusJson()
                               : "{\"op\":\"error\",\"message\":\"record_start failed\"}");
  } else if (op == "record_stop") {
    recorder_.StopRecording();
    websocket_->SendText(client_id, recorder_.StatusJson());
  } else if (op == "record_set_filter") {
    recorder_.SetChannelFilter(ExtractJsonStringArray(text, "channels"));
    websocket_->SendText(client_id, recorder_.StatusJson());
  } else if (op == "playback_start") {
    PlaybackOptions po;
    po.path = ExtractJsonString(text, "path");
    if (po.path.empty()) po.path = "/tmp/orbisview_record.jsonl";
    po.speed = ExtractJsonNumber(text, "speed", 1.0);
    po.loop = ExtractJsonBool(text, "loop", false);
    po.start_index =
        static_cast<uint64_t>(ExtractJsonNumber(text, "start_index", 0.0));
    po.start_timestamp_ns =
        static_cast<int64_t>(ExtractJsonNumber(text, "start_timestamp", 0.0));
    const bool ok = recorder_.StartPlayback(
        po, [this](core::StreamEnvelope env) { EmitEnvelope(std::move(env)); });
    websocket_->SendText(client_id, ok ? recorder_.StatusJson()
                               : "{\"op\":\"error\",\"message\":\"playback_start failed\"}");
  } else if (op == "playback_stop") {
    recorder_.StopPlayback();
    websocket_->SendText(client_id, recorder_.StatusJson());
  } else if (op == "playback_pause") {
    recorder_.PausePlayback(ExtractJsonBool(text, "paused", true));
    websocket_->SendText(client_id, recorder_.StatusJson());
  } else if (op == "playback_seek") {
    const double idx = ExtractJsonNumber(text, "index", -1.0);
    const double ts = ExtractJsonNumber(text, "timestamp", -1.0);
    bool ok = false;
    if (idx >= 0) {
      ok = recorder_.SeekIndex(static_cast<uint64_t>(idx));
    } else if (ts >= 0) {
      ok = recorder_.SeekTimestamp(static_cast<int64_t>(ts));
    }
    websocket_->SendText(client_id, ok ? recorder_.StatusJson()
                               : "{\"op\":\"error\",\"message\":\"playback_seek failed\"}");
  } else if (op == "bag_index") {
    const auto path = ExtractJsonString(text, "path");
    const std::string bag =
        path.empty() ? "/tmp/orbisview_record.jsonl" : path;
    if (!recorder_.EnsureIndex(bag)) {
      websocket_->SendText(client_id,
                   "{\"op\":\"error\",\"message\":\"bag_index failed\"}");
    } else {
      websocket_->SendText(client_id, recorder_.IndexJson(bag));
    }
  } else if (op == "set_goal") {
    const double x = ExtractJsonNumber(text, "x", 0.0);
    const double y = ExtractJsonNumber(text, "y", 0.0);
    const double yaw = ExtractJsonNumber(text, "yaw", 0.0);
    if (options_.enable_mock) {
      mock_.SetNavGoal(x, y, yaw);
    }
    std::ostringstream ack;
    ack << "{\"op\":\"goal_set\",\"x\":" << x << ",\"y\":" << y
        << ",\"yaw\":" << yaw
        << ",\"mock\":" << (options_.enable_mock ? "true" : "false") << '}';
    websocket_->SendText(client_id, ack.str());
  } else if (op == "clear_goal") {
    if (options_.enable_mock) mock_.ClearNavGoal();
    websocket_->SendText(client_id, "{\"op\":\"goal_cleared\"}");
  } else if (op == "cmd_vel") {
    const double vx = ExtractJsonNumber(text, "vx", 0.0);
    const double wz = ExtractJsonNumber(text, "wz", 0.0);
    // TeleopService publisher (Autolink) + optional mock chassis.
    if (teleop_) teleop_->SetCmdVel(vx, wz);
    if (options_.enable_mock) {
      mock_.SetCmdVel(vx, wz);
    }
    std::ostringstream ack;
    ack << "{\"op\":\"cmd_vel_ack\",\"vx\":" << vx << ",\"wz\":" << wz << '}';
    websocket_->SendText(client_id, ack.str());
  } else if (op == "set_route") {
    auto wps = ExtractWaypoints(text);
    if (options_.enable_mock) {
      mock_.SetRoute(wps);
      if (!wps.empty()) {
        const auto& last = wps.back();
        mock_.SetNavGoal(last.x, last.y, last.yaw);
      }
    }
    std::ostringstream ack;
    ack << "{\"op\":\"route_set\",\"count\":" << wps.size() << '}';
    websocket_->SendText(client_id, ack.str());
  } else if (op == "clear_route") {
    if (options_.enable_mock) mock_.ClearRoute();
    websocket_->SendText(client_id, "{\"op\":\"route_cleared\"}");
  } else if (op == "hmi_set_mode") {
    const auto mode = ExtractJsonString(text, "mode");
    const bool ok = hmi_->SetMode(mode);
    websocket_->SendText(client_id,
                 ok ? ("{\"op\":\"hmi_mode\",\"mode\":\"" + mode + "\"}")
                    : "{\"op\":\"error\",\"message\":\"unknown hmi mode\"}");
  } else if (op == "hmi_module_action") {
    const auto id = ExtractJsonString(text, "id");
    const auto action = ExtractJsonString(text, "action");
    const bool ok = hmi_->ModuleAction(id, action);
    websocket_->SendText(client_id,
                 ok ? "{\"op\":\"hmi_module_ack\"}"
                    : "{\"op\":\"error\",\"message\":\"hmi module action "
                      "failed\"}");
  } else if (op == "hmi_status") {
    websocket_->SendText(client_id,
                 std::string("{\"op\":\"hmi_status\",\"status\":") +
                     hmi_->StatusJson() +
                     ",\"components\":" + hmi_->ComponentsJson() + '}');
  } else if (op == "dump_snapshot") {
    auto path = ExtractJsonString(text, "path");
    if (path.empty()) path = "/tmp/orbisview_dump.json";
    std::ofstream ofs(path);
    if (!ofs) {
      websocket_->SendText(client_id,
                   "{\"op\":\"error\",\"message\":\"dump_snapshot open failed\"}");
    } else {
      ofs << "{\"world\":" << sim_world_updater_->WorldJson()
          << ",\"hmi\":" << hmi_->StatusJson()
          << ",\"components\":" << hmi_->ComponentsJson() << '}';
      websocket_->SendText(client_id,
                   std::string("{\"op\":\"dump_ok\",\"path\":\"") + path + "\"}");
    }
  } else if (op == "clear_sim") {
    if (options_.enable_mock) {
      mock_.ClearNavGoal();
      mock_.ClearRoute();
    }
    sim_world_updater_->Service()->ClearGoal();
    websocket_->SendText(client_id, "{\"op\":\"sim_cleared\"}");
  } else if (op == "list_local_bags") {
    auto dir = ExtractJsonString(text, "path");
    if (dir.empty()) dir = "/tmp";
    std::ostringstream oss;
    oss << "{\"op\":\"local_bags\",\"path\":\"" << dir << "\",\"bags\":[";
    bool first = true;
#if defined(_WIN32)
    (void)dir;
#else
    DIR* d = opendir(dir.c_str());
    if (d) {
      while (auto* ent = readdir(d)) {
        const std::string name = ent->d_name;
        if (name.size() > 6 && name.substr(name.size() - 6) == ".jsonl") {
          if (!first) oss << ',';
          first = false;
          oss << "{\"name\":\"" << name << "\",\"path\":\"" << dir << '/'
              << name << "\"}";
        }
      }
      closedir(d);
    }
#endif
    oss << "]}";
    websocket_->SendText(client_id, oss.str());
  } else {
    websocket_->SendText(client_id, "{\"op\":\"error\",\"message\":\"unknown op\"}");
  }
}

std::vector<core::ChannelInfo> Orbisview::CollectChannels() const {
  std::vector<core::ChannelInfo> out;
  if (options_.enable_mock) {
    auto mock_channels = mock_.Channels();
    out.insert(out.end(), mock_channels.begin(), mock_channels.end());
  }
#if defined(ORBISVIEW_WITH_AUTOLINK)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    out.insert(out.end(), autolink_channels_.begin(), autolink_channels_.end());
  }
#endif
  return out;
}

void Orbisview::HandleListChannels(int client_id) {
#if defined(ORBISVIEW_WITH_AUTOLINK)
  if (options_.enable_autolink) RefreshAutolinkChannels();
#endif
  websocket_->SendText(client_id, core::ChannelListToJson(CollectChannels()));
}

void Orbisview::BroadcastChannels() {
  websocket_->BroadcastText(core::ChannelListToJson(CollectChannels()));
}

void Orbisview::HandleSubscribe(int client_id, const std::string& channel,
                                      double max_hz) {
  if (channel.empty()) {
    websocket_->SendText(client_id, "{\"op\":\"error\",\"message\":\"missing channel\"}");
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    core::SubscriptionState state;
    state.options.max_hz = max_hz;
    subs_[client_id][channel] = state;
  }
#if defined(ORBISVIEW_WITH_AUTOLINK)
  if (options_.enable_autolink) EnsureAutolinkSubscribe(channel);
#endif
  std::ostringstream ack;
  ack << "{\"op\":\"subscribed\",\"channel\":" << core::JsonEscape(channel)
      << ",\"max_hz\":" << max_hz << '}';
  websocket_->SendText(client_id, ack.str());
}

void Orbisview::HandleUnsubscribe(int client_id,
                                        const std::string& channel) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = subs_.find(client_id);
    if (it != subs_.end()) it->second.erase(channel);
  }
  std::ostringstream ack;
  ack << "{\"op\":\"unsubscribed\",\"channel\":" << core::JsonEscape(channel)
      << '}';
  websocket_->SendText(client_id, ack.str());
}

void Orbisview::HandleStatus(int client_id) {
  std::ostringstream oss;
  oss << "{\"op\":\"status\",\"clients\":" << websocket_->ClientCount()
      << ",\"dropped_frames\":" << outbound_.DroppedFrames()
      << ",\"mock\":" << (options_.enable_mock ? "true" : "false")
      << ",\"autolink\":" << (options_.enable_autolink ? "true" : "false")
      << ",\"transport\":\"civetweb\"}";
  websocket_->SendText(client_id, oss.str());
}

void Orbisview::EmitEnvelope(core::StreamEnvelope env) {
  IngestWorld(env);
  if (hmi_) hmi_->UpdateChannelHealth(env.channel, 0.0, true);
  stats_.Observe(env);
  recorder_.Append(env);
  outbound_.Push(std::move(env));
}

void Orbisview::IngestWorld(const core::StreamEnvelope& env) {
  if (sim_world_updater_) sim_world_updater_->Ingest(env);
}
bool Orbisview::ShouldForward(int client_id, const std::string& channel,
                                    core::SubscriptionState* state) {
  auto client_it = subs_.find(client_id);
  if (client_it == subs_.end()) return false;
  auto sub_it = client_it->second.find(channel);
  if (sub_it == client_it->second.end()) return false;
  *state = sub_it->second;
  if (state->options.max_hz > 0.0) {
    const auto now = std::chrono::steady_clock::now();
    const auto min_dt =
        std::chrono::duration<double>(1.0 / state->options.max_hz);
    if (state->last_send.time_since_epoch().count() != 0 &&
        (now - state->last_send) < min_dt) {
      return false;
    }
    sub_it->second.last_send = now;
  }
  ++sub_it->second.delivered;
  return true;
}

void Orbisview::PumpLoop() {
  last_topo_refresh_ = std::chrono::steady_clock::now();
  while (running_.load()) {
#if defined(ORBISVIEW_WITH_AUTOLINK)
    if (options_.enable_autolink) {
      const auto now = std::chrono::steady_clock::now();
      if (now - last_topo_refresh_ > std::chrono::seconds(2)) {
        last_topo_refresh_ = now;
        if (RefreshAutolinkChannelsChanged()) {
          BroadcastChannels();
        }
      }
    }
#endif
    auto env = outbound_.Pop();
    if (!env) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      continue;
    }
    const std::string json = core::StreamEnvelopeToJson(*env);
    std::vector<int> clients;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (const auto& kv : subs_) clients.push_back(kv.first);
    }
    for (int id : clients) {
      core::SubscriptionState state;
      bool ok = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        ok = ShouldForward(id, env->channel, &state);
      }
      if (ok) websocket_->SendText(id, json);
    }
  }
}

#if defined(ORBISVIEW_WITH_AUTOLINK)
void Orbisview::RefreshAutolinkChannels() {
  (void)RefreshAutolinkChannelsChanged();
}

bool Orbisview::RefreshAutolinkChannelsChanged() {
  auto* topo = autolink::service_discovery::TopologyManager::Instance();
  if (!topo || !topo->channel_manager()) return false;
  auto cm = topo->channel_manager();
  std::vector<std::string> names;
  cm->GetChannelNames(&names);
  std::vector<core::ChannelInfo> channels;
  std::ostringstream fp;
  for (const auto& name : names) {
    if (!cm->HasWriter(name)) continue;
    std::string msg_type;
    cm->GetMsgType(name, &msg_type);
#if defined(ORBISVIEW_WITH_AUTOMSGS)
    const std::string schema = adapters::SuggestedRenderSchema(msg_type);
#else
    const std::string schema;
#endif
    const std::string effective_schema =
        schema.empty() ? (msg_type.empty() ? "unsupported" : msg_type) : schema;
    channels.push_back({name, effective_schema, msg_type, true, false});
    fp << name << '|' << msg_type << ';';
  }
  const std::string fingerprint = fp.str();
  std::lock_guard<std::mutex> lock(mutex_);
  if (fingerprint == autolink_fingerprint_) return false;
  autolink_fingerprint_ = fingerprint;
  autolink_channels_ = std::move(channels);
  return true;
}

void Orbisview::EnsureAutolinkSubscribe(const std::string& channel) {
  if (!node_ || readers_.count(channel)) return;

  auto reader = node_->CreateReader<autolink::message::RawMessage>(
      channel,
      [this, channel](
          const std::shared_ptr<autolink::message::RawMessage>& message) {
        if (!message) return;
        std::string msg_type;
        auto* topo = autolink::service_discovery::TopologyManager::Instance();
        if (topo && topo->channel_manager()) {
          topo->channel_manager()->GetMsgType(channel, &msg_type);
        }
        core::StreamEnvelope env;
        env.channel = channel;
        env.timestamp_ns = static_cast<int64_t>(message->timestamp);
        env.sequence = 0;
        bool converted = false;
#if defined(ORBISVIEW_WITH_AUTOMSGS)
        converted = adapters::ConvertAutomsgsRaw(
            channel, msg_type, message->message, env.timestamp_ns, &env);
#endif
        if (!converted) {
          env.schema = msg_type.empty() ? "autolink.raw" : msg_type;
          env.encoding = "protobuf";
          env.payload.assign(message->message.begin(), message->message.end());
          env.unsupported = true;
        }
        EmitEnvelope(std::move(env));
      });
  readers_[channel] = reader;
  RefreshAutolinkChannels();
}

void Orbisview::PublishCmdVelAutolink(double vx, double wz) {
  if (!node_ || options_.cmd_vel_channel.empty()) return;
#if defined(ORBISVIEW_WITH_AUTOMSGS)
  // autosim / nav stack subscribe TwistStamped on /cmd_vel (not Twist2D).
  using TwistStamped = automsgs::msgs::geometry_msgs::TwistStamped;
  if (!cmd_vel_writer_) {
    cmd_vel_writer_ = node_->CreateWriter<TwistStamped>(options_.cmd_vel_channel);
  }
  auto writer =
      std::static_pointer_cast<autolink::Writer<TwistStamped>>(cmd_vel_writer_);
  if (!writer) return;
  auto msg = std::make_shared<TwistStamped>();
  auto* header = msg->mutable_header();
  header->set_frame_id("base_link");
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  const auto sec = std::chrono::duration_cast<std::chrono::seconds>(now);
  const auto nsec =
      std::chrono::duration_cast<std::chrono::nanoseconds>(now - sec);
  auto* stamp = header->mutable_stamp();
  stamp->set_sec(static_cast<int32_t>(sec.count()));
  stamp->set_nanosec(static_cast<uint32_t>(nsec.count()));
  auto* twist = msg->mutable_twist();
  twist->mutable_linear()->set_x(vx);
  twist->mutable_linear()->set_y(0.0);
  twist->mutable_linear()->set_z(0.0);
  twist->mutable_angular()->set_x(0.0);
  twist->mutable_angular()->set_y(0.0);
  twist->mutable_angular()->set_z(wz);
  writer->Write(msg);
#else
  (void)vx;
  (void)wz;
  LOG(WARNING) << "cmd_vel Autolink publish requires ORBISVIEW_WITH_AUTOMSGS";
#endif
}
#endif

}  // namespace backend
}  // namespace orbisview
}  // namespace autonomy
