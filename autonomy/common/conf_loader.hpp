/*
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>

#include "autolink/common/file.hpp"
#include "autolink/common/environment.hpp"
#include "autonomy/common/config.hpp"
#include "autonomy/common/logging.hpp"
#include "google/protobuf/message.h"

namespace autonomy {
namespace common {

/** Distribution root: explicit override, executable's install tree, configured prefix. */
inline std::string AutonomyWorkRoot() {
  const char* env = std::getenv("AUTONOMY_PATH");
  if (env != nullptr && env[0] != '\0') {
    return std::string(env);
  }
  const auto installed = autolink::common::InstalledPrefix();
  return installed.empty() ? std::string(kLibraryInstallDir) : installed;
}

/**
 * Resolve a module conf file (autolink LoadConfig style).
 *
 * Search order:
 *  1. Absolute path / path existing relative to CWD
 *  2. AUTONOMY_CONFIG_DIR/<module>/<file> (explicit user config root)
 *  3. AUTONOMY_CONF_PATH (colon-separated roots; append module/conf/file)
 *  4. ~/.autonomy/config/<module>/<file> (when CONFIG_DIR is unset)
 *  5. Distribution and source-tree defaults
 */
inline bool ResolveModuleConfPath(const std::string& module,
                                  const std::string& conf_file,
                                  std::string* resolved) {
  CHECK_NOTNULL(resolved);
  if (conf_file.empty()) {
    return false;
  }

  const std::string relative = module.empty()
                                   ? ("conf/" + conf_file)
                                   : (module + "/conf/" + conf_file);

  if (autolink::common::PathIsAbsolute(conf_file) ||
      conf_file.find('/') != std::string::npos) {
    if (autolink::common::GetFilePathWithEnv(conf_file, "AUTONOMY_CONF_PATH",
                                             resolved)) {
      return true;
    }
  }

  const char* config_dir = std::getenv("AUTONOMY_CONFIG_DIR");
  const bool explicit_config_dir = config_dir != nullptr && config_dir[0] != '\0';
  if (explicit_config_dir) {
    const auto path = std::filesystem::path(config_dir) / module / conf_file;
    if (std::filesystem::is_regular_file(path)) {
      *resolved = path.string();
      return true;
    }
  }

  if (autolink::common::GetFilePathWithEnv(relative, "AUTONOMY_CONF_PATH",
                                           resolved)) {
    return true;
  }

  if (!explicit_config_dir) {
    const char* home = std::getenv("HOME");
    if (home != nullptr && home[0] != '\0') {
      const auto path = std::filesystem::path(home) / ".autonomy/config" / module / conf_file;
      if (std::filesystem::is_regular_file(path)) {
        *resolved = path.string();
        return true;
      }
    }
  }

  const std::vector<std::string> roots = {
      AutonomyWorkRoot(),
      std::string(kSourceDirectory),
      std::string(kLibraryInstallDir),
  };
  for (const auto& root : roots) {
    if (root.empty()) {
      continue;
    }
    const std::string share =
        root + "/share/autonomy/" + module + "/conf/" + conf_file;
    if (autolink::common::PathExists(share)) {
      *resolved = share;
      return true;
    }
    const std::string src =
        root + "/autonomy/" + module + "/conf/" + conf_file;
    if (autolink::common::PathExists(src)) {
      *resolved = src;
      return true;
    }
  }
  return false;
}

/** Load protobuf text/binary conf for a module (like autolink::LoadConfig). */
template <typename T>
bool LoadModuleConf(const std::string& module, const std::string& conf_file,
                    T* config) {
  CHECK_NOTNULL(config);
  std::string path;
  if (!ResolveModuleConfPath(module, conf_file, &path)) {
    LOG(ERROR) << "conf not found: module=" << module
               << " file=" << conf_file
               << " (set AUTONOMY_PATH or AUTONOMY_CONF_PATH)";
    return false;
  }
  LOG(INFO) << "load conf: " << path;
  return autolink::common::GetProtoFromFile(path, config);
}

/** Resolve a non-proto asset under autonomy/<module>/conf/ (BT xml, maps, …). */
inline bool ResolveModuleAsset(const std::string& module,
                               const std::string& relative_under_conf,
                               std::string* resolved) {
  return ResolveModuleConfPath(module, relative_under_conf, resolved);
}

}  // namespace common
}  // namespace autonomy
