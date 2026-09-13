/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/orbisview/backend/common/plugins/plugin_host.h"

#include <dlfcn.h>

#include <filesystem>
#include <sstream>

#include <glog/logging.h>

#include "autonomy/orbisview/backend/common/stream_envelope.h"

namespace autonomy {
namespace orbisview {
namespace plugins {
namespace {

void HostLogInfo(const char* msg) {
  if (msg) LOG(INFO) << "[plugin] " << msg;
}

void HostLogError(const char* msg) {
  if (msg) LOG(ERROR) << "[plugin] " << msg;
}

bool LooksLikeSharedLib(const std::filesystem::path& p) {
  const auto ext = p.extension().string();
  return ext == ".so" || ext == ".dylib";
}

}  // namespace

PluginHost::PluginHost(PluginRegistry* registry) : registry_(registry) {}

PluginHost::~PluginHost() {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<std::string> ids;
  ids.reserve(loaded_.size());
  for (const auto& kv : loaded_) ids.push_back(kv.first);
  for (const auto& id : ids) UnloadLocked(id);
}

int PluginHost::ScanAndLoad(const std::string& directory) {
  namespace fs = std::filesystem;
  std::error_code ec;
  if (!fs::is_directory(directory, ec)) {
    std::lock_guard<std::mutex> lock(mutex_);
    load_failures_.push_back("scan: not a directory: " + directory);
    return 0;
  }
  int loaded = 0;
  for (const auto& entry : fs::directory_iterator(directory, ec)) {
    if (ec) break;
    if (!entry.is_regular_file(ec)) continue;
    if (!LooksLikeSharedLib(entry.path())) continue;
    if (Load(entry.path().string())) ++loaded;
  }
  return loaded;
}

bool PluginHost::Load(const std::string& path) {
  std::lock_guard<std::mutex> lock(mutex_);
  return LoadLocked(path);
}

bool PluginHost::Unload(const std::string& id) {
  std::lock_guard<std::mutex> lock(mutex_);
  return UnloadLocked(id);
}

bool PluginHost::Reload(const std::string& id) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = loaded_.find(id);
  if (it == loaded_.end()) {
    load_failures_.push_back("reload: not loaded: " + id);
    return false;
  }
  const std::string path = it->second.path;
  if (!UnloadLocked(id)) return false;
  return LoadLocked(path);
}

bool PluginHost::LoadLocked(const std::string& path) {
  // Already loaded from this path?
  for (const auto& kv : loaded_) {
    if (kv.second.path == path) {
      load_failures_.push_back("already loaded: " + path);
      return false;
    }
  }

  void* handle = dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
  if (!handle) {
    const char* err = dlerror();
    load_failures_.push_back(std::string("dlopen failed: ") + path + " : " +
                             (err ? err : "unknown"));
    LOG(ERROR) << load_failures_.back();
    return false;
  }

  dlerror();
  auto* reg = reinterpret_cast<OrbisPluginRegisterFn>(
      dlsym(handle, "orbisview_plugin_register"));
  const char* sym_err = dlerror();
  if (sym_err || !reg) {
    load_failures_.push_back(std::string("missing orbisview_plugin_register: ") +
                             path + " : " + (sym_err ? sym_err : "null"));
    dlclose(handle);
    LOG(ERROR) << load_failures_.back();
    return false;
  }

  OrbisPluginHostApi host_api{};
  host_api.abi_version = ORBISVIEW_PLUGIN_ABI_VERSION;
  host_api.log_info = &HostLogInfo;
  host_api.log_error = &HostLogError;

  OrbisPluginInfo info{};
  int rc = -1;
  try {
    rc = reg(&host_api, &info);
  } catch (...) {
    load_failures_.push_back("plugin register threw: " + path);
    dlclose(handle);
    LOG(ERROR) << load_failures_.back();
    return false;
  }

  if (rc != 0 || !info.id || info.id[0] == '\0') {
    load_failures_.push_back("plugin register failed: " + path +
                             " rc=" + std::to_string(rc));
    dlclose(handle);
    LOG(ERROR) << load_failures_.back();
    return false;
  }

  PluginInfo pi;
  pi.id = info.id;
  pi.kind = info.kind ? info.kind : "tool";
  pi.title = info.title ? info.title : info.id;
  pi.version = info.version ? info.version : "0.0";
  pi.enabled = true;
  pi.source = "dynamic";
  pi.path = path;

  if (loaded_.count(pi.id)) {
    load_failures_.push_back("duplicate plugin id after load: " + pi.id);
    dlclose(handle);
    return false;
  }

  if (registry_ && !registry_->Register(pi)) {
    load_failures_.push_back("registry reject: " + pi.id);
    dlclose(handle);
    return false;
  }

  LoadedPlugin lp;
  lp.path = path;
  lp.handle = handle;
  lp.info = pi;
  lp.unregister = reinterpret_cast<OrbisPluginUnregisterFn>(
      dlsym(handle, "orbisview_plugin_unregister"));
  loaded_[pi.id] = lp;
  LOG(INFO) << "OrbisView plugin loaded: " << pi.id << " from " << path;
  return true;
}

bool PluginHost::UnloadLocked(const std::string& id) {
  auto it = loaded_.find(id);
  if (it == loaded_.end()) {
    load_failures_.push_back("unload: not found: " + id);
    return false;
  }
  if (it->second.unregister) {
    try {
      it->second.unregister();
    } catch (...) {
      LOG(WARNING) << "plugin unregister threw: " << id;
    }
  }
  if (registry_) registry_->Unregister(id);
  if (it->second.handle) dlclose(it->second.handle);
  loaded_.erase(it);
  LOG(INFO) << "OrbisView plugin unloaded: " << id;
  return true;
}

std::string PluginHost::StatusJson() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::ostringstream oss;
  oss << "{\"op\":\"plugin_host\",\"loaded\":[";
  bool first = true;
  for (const auto& kv : loaded_) {
    if (!first) oss << ',';
    first = false;
    const auto& p = kv.second;
    oss << "{\"id\":" << core::JsonEscape(p.info.id)
        << ",\"path\":" << core::JsonEscape(p.path)
        << ",\"kind\":" << core::JsonEscape(p.info.kind)
        << ",\"version\":" << core::JsonEscape(p.info.version) << '}';
  }
  oss << "],\"failures\":[";
  for (size_t i = 0; i < load_failures_.size(); ++i) {
    if (i) oss << ',';
    oss << core::JsonEscape(load_failures_[i]);
  }
  oss << "]}";
  return oss.str();
}

}  // namespace plugins
}  // namespace orbisview
}  // namespace autonomy
