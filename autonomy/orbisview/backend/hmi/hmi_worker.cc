/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/orbisview/backend/hmi/hmi_worker.h"

#include <fstream>
#include <sstream>

#include "autonomy/orbisview/backend/common/util/json_util.h"

#if !defined(_WIN32)
#include <dirent.h>
#endif

namespace autonomy {
namespace orbisview {
namespace core {

HmiWorker::HmiWorker() { EnsureBuiltinModes(); }

void HmiWorker::EnsureBuiltinModes() {
  modes_ = {
      {"default",
       "Default",
       {"localization", "perception", "planning", "control"},
       {"/orbisview/mock/pose", "/orbisview/mock/laser"}},
      {"pnc",
       "PNC",
       {"localization", "planning", "control", "prediction"},
       {"/orbisview/mock/path", "/orbisview/mock/planning"}},
      {"mapping",
       "Mapping",
       {"localization", "mapping"},
       {"/orbisview/mock/map", "/orbisview/mock/mapping"}},
  };
  for (const auto& id :
       {"localization", "perception", "planning", "control", "prediction",
        "mapping"}) {
    modules_[id] = {id, id, true, false, 0};
  }
}

void HmiWorker::LoadModesDir(const std::string& dir) {
  if (dir.empty()) return;
#if !defined(_WIN32)
  DIR* d = opendir(dir.c_str());
  if (!d) return;
  while (auto* ent = readdir(d)) {
    const std::string name = ent->d_name;
    if (name.size() < 6 || name.substr(name.size() - 5) != ".json") continue;
    std::ifstream in(dir + "/" + name);
    if (!in) continue;
    std::stringstream buf;
    buf << in.rdbuf();
    const std::string text = buf.str();
    HmiMode mode;
    mode.id = util::ExtractJsonString(text, "id");
    mode.title = util::ExtractJsonString(text, "title");
    if (mode.id.empty()) continue;
    if (mode.title.empty()) mode.title = mode.id;
    // module_ids / channel_hints optional — keep empty if absent
    std::lock_guard<std::mutex> lock(mutex_);
    bool replaced = false;
    for (auto& m : modes_) {
      if (m.id == mode.id) {
        m = mode;
        replaced = true;
        break;
      }
    }
    if (!replaced) modes_.push_back(mode);
  }
  closedir(d);
#endif
}

std::vector<HmiMode> HmiWorker::Modes() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return modes_;
}

std::string HmiWorker::CurrentModeId() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return current_mode_;
}

bool HmiWorker::SetMode(const std::string& mode_id) {
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& m : modes_) {
    if (m.id == mode_id) {
      current_mode_ = mode_id;
      for (auto& kv : modules_) kv.second.expected = false;
      for (const auto& mid : m.module_ids) {
        modules_[mid].expected = true;
        modules_[mid].healthy = true;
      }
      return true;
    }
  }
  return false;
}

bool HmiWorker::ModuleAction(const std::string& module_id,
                             const std::string& action) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = modules_.find(module_id);
  if (it == modules_.end()) return false;
  if (action == "start" || action == "enable") {
    it->second.expected = true;
    it->second.healthy = true;
  } else if (action == "stop" || action == "disable") {
    it->second.expected = false;
    it->second.healthy = false;
  } else {
    return false;
  }
  return true;
}

void HmiWorker::UpdateChannelHealth(const std::string& channel,
                                    double delay_ms, bool alive) {
  std::lock_guard<std::mutex> lock(mutex_);
  std::string module;
  if (channel.find("pose") != std::string::npos ||
      channel.find("odom") != std::string::npos) {
    module = "localization";
  } else if (channel.find("laser") != std::string::npos ||
             channel.find("obstacle") != std::string::npos ||
             channel.find("pointcloud") != std::string::npos) {
    module = "perception";
  } else if (channel.find("path") != std::string::npos ||
             channel.find("planning") != std::string::npos ||
             channel.find("route") != std::string::npos) {
    module = "planning";
  } else if (channel.find("twist") != std::string::npos ||
             channel.find("chassis") != std::string::npos ||
             channel.find("cmd_vel") != std::string::npos) {
    module = "control";
  } else if (channel.find("prediction") != std::string::npos) {
    module = "prediction";
  } else if (channel.find("map") != std::string::npos ||
             channel.find("mapping") != std::string::npos) {
    module = "mapping";
  }
  if (module.empty()) return;
  auto& m = modules_[module];
  m.id = module;
  m.title = module;
  m.delay_ms = delay_ms;
  if (alive) m.healthy = true;
}

std::string HmiWorker::StatusJson() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::ostringstream oss;
  oss << "{\"mode\":\"" << current_mode_ << "\",\"modes\":[";
  for (size_t i = 0; i < modes_.size(); ++i) {
    if (i) oss << ',';
    oss << "{\"id\":\"" << modes_[i].id << "\",\"title\":\"" << modes_[i].title
        << "\"}";
  }
  oss << "]}";
  return oss.str();
}

std::string HmiWorker::ComponentsJson() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::ostringstream oss;
  oss << "{\"components\":[";
  bool first = true;
  for (const auto& kv : modules_) {
    if (!first) oss << ',';
    first = false;
    const auto& m = kv.second;
    const char* status =
        !m.expected ? "DISABLED" : (m.healthy ? "OK" : "ERROR");
    oss << "{\"id\":\"" << m.id << "\",\"title\":\"" << m.title
        << "\",\"expected\":" << (m.expected ? "true" : "false")
        << ",\"healthy\":" << (m.healthy ? "true" : "false")
        << ",\"delay_ms\":" << m.delay_ms << ",\"status\":\"" << status
        << "\"}";
  }
  oss << "]}";
  return oss.str();
}

}  // namespace core
}  // namespace orbisview
}  // namespace autonomy
