/*
 * Copyright 2026 The Openbot Authors
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

#include "autonomy/common/logging.hpp"
#include "autonomy/localization/cartographer/node/node_utils.hpp"

#include <algorithm>
#include <cstdlib>
#include <csignal>
#include <filesystem>
#include <string>
#include <vector>

#include <glog/logging.h>

#include "autolink/autolink.hpp"
#include "autonomy/transform/buffer_utils.hpp"

namespace autonomy {
namespace localization {
namespace cartographer {
namespace node {
namespace {

namespace fs = std::filesystem;

void SigintHandler(int /*sig*/) { autolink::AsyncShutdown(); }

std::string StaticTransformYamlPath(const std::string& configuration_directory,
                                    const std::string& configuration_basename) {
    std::string stem = configuration_basename;
    constexpr const char* kLuaSuffix = ".lua";
    if (stem.size() > 4 &&
        stem.compare(stem.size() - 4, 4, kLuaSuffix) == 0) {
        stem.erase(stem.size() - 4);
    }
    return configuration_directory + "/" + stem + "_static_transform.yaml";
}

}  // namespace

int64_t TimerPeriodMs(const double seconds) {
    return static_cast<int64_t>(std::max(seconds * 1000.0, 1.0));
}

std::string ResolveWorkspacePath(const std::string& path) {
    if (path.empty()) {
        return path;
    }

    const fs::path relative(path);
    // Short profile paths like conf/atlas/... live under autonomy/localization/.
    std::vector<fs::path> candidates = {relative};
    if (path.rfind("conf/", 0) == 0) {
        candidates.emplace_back(fs::path("autonomy/localization") / relative);
    }

    auto try_exists = [](const fs::path& p) -> std::string {
        if (fs::exists(p)) {
            return fs::weakly_canonical(p).string();
        }
        return {};
    };

    for (const auto& cand : candidates) {
        if (auto hit = try_exists(cand); !hit.empty()) {
            return hit;
        }
    }

    if (const char* dev_dir = std::getenv("AUTONOMY_DEV_DIR")) {
        for (const auto& cand : candidates) {
            if (auto hit = try_exists(fs::path(dev_dir) / cand); !hit.empty()) {
                return hit;
            }
        }
    }

    fs::path cwd = fs::current_path();
    for (int depth = 0; depth < 10; ++depth) {
        for (const auto& cand : candidates) {
            if (auto hit = try_exists(cwd / cand); !hit.empty()) {
                return hit;
            }
        }
        if (!cwd.has_parent_path() || cwd == cwd.root_path()) {
            break;
        }
        cwd = cwd.parent_path();
    }

    return path;
}

void RegisterAutolinkShutdownHandlers() {
    signal(SIGINT, SigintHandler);
    signal(SIGTERM, SigintHandler);
}

std::string ResolveStaticTransformYamlPath(
    const std::string& configuration_directory,
    const std::string& configuration_basename) {
    return ResolveWorkspacePath(
        StaticTransformYamlPath(configuration_directory, configuration_basename));
}

void LoadStaticTransformsForConfig(
    const std::string& configuration_directory,
    const std::string& configuration_basename) {
    const std::string yaml_path =
        ResolveStaticTransformYamlPath(configuration_directory,
                                       configuration_basename);
    if (!fs::exists(yaml_path)) {
        LOG(WARNING) << "Static transform file not found, skip: " << yaml_path;
        return;
    }

    transform::LoadStaticTransformsFromFile(transform::Buffer::Instance(),
                                            yaml_path, "static_transform");
}

}  // namespace node
}  // namespace cartographer
}  // namespace localization
}  // namespace autonomy
