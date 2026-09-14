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
#include <string>
#include <vector>

#include "autolink/common/file.hpp"
#include "autonomy/common/config.hpp"
#include "autonomy/common/logging.hpp"
#include "google/protobuf/message.h"

namespace autonomy {
namespace common {

/** Install / source distribution root (AUTONOMY_PATH, else install prefix). */
inline std::string AutonomyWorkRoot() {
  const char* env = std::getenv("AUTONOMY_PATH");
  if (env != nullptr && env[0] != '\0') {
    return std::string(env);
  }
  return std::string(kLibraryInstallDir);
}

/**
 * Resolve a module conf file (autolink LoadConfig style).
 *
 * Search order:
 *  1. Absolute path / path existing relative to CWD
 *  2. AUTONOMY_CONF_PATH (colon-separated roots; append module/conf/file)
 *  3. $AUTONOMY_PATH/share/autonomy/<module>/conf/<file>
 *  4. $AUTONOMY_PATH/autonomy/<module>/conf/<file> (source-tree layout)
 *  5. kSourceDirectory/autonomy/<module>/conf/<file>
 *  6. kLibraryInstallDir/share/autonomy/<module>/conf/<file>
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

  if (autolink::common::GetFilePathWithEnv(relative, "AUTONOMY_CONF_PATH",
                                           resolved)) {
    return true;
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
