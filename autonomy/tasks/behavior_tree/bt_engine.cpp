/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include "autonomy/tasks/behavior_tree/bt_engine.hpp"

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "autonomy/common/config.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/tasks/behavior_tree/json_utils.hpp"
#include "behaviortree_cpp/json_export.h"
#include "behaviortree_cpp/utils/shared_library.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace {

std::string PluginLibraryFilename(const std::string& plugin_name) {
    BT::SharedLibrary loader;
    return loader.getOSName(plugin_name);
}

std::vector<std::filesystem::path> PluginSearchDirectories(
    const std::string& plugin_lib_path) {
    std::vector<std::filesystem::path> dirs;
    if (!plugin_lib_path.empty()) {
        dirs.emplace_back(plugin_lib_path);
    }
    dirs.emplace_back(std::string(autonomy::common::kLibraryBuildDir) + "/lib");
    if (const char* env = std::getenv("AUTONOMY_BT_PLUGIN_PATH");
        env != nullptr && env[0] != '\0') {
        dirs.emplace_back(env);
    }
    return dirs;
}

std::string ResolvePluginLibraryPath(const std::string& plugin_name,
                                     const std::string& plugin_lib_path) {
    const std::string filename = PluginLibraryFilename(plugin_name);
    for (const auto& dir : PluginSearchDirectories(plugin_lib_path)) {
        const auto candidate = dir / filename;
        if (std::filesystem::exists(candidate)) {
            return candidate.string();
        }
    }
    return filename;
}

}  // namespace

BtEngine::BtEngine(const std::vector<std::string>& plugin_libraries,
                   const std::string& plugin_lib_path) {
    for (const auto& p : plugin_libraries) {
        const std::string lib_path =
            ResolvePluginLibraryPath(p, plugin_lib_path);
        try {
            factory_.registerFromPlugin(lib_path);
        } catch (const std::exception& ex) {
            AERROR << "Failed to load BT plugin '" << p << "' from '"
                   << lib_path << "': " << ex.what();
            throw;
        }
    }

    BT::ReactiveSequence::EnableException(false);
    BT::ReactiveFallback::EnableException(false);
}

BtStatus BtEngine::Run(BT::Tree* tree, std::function<void()> onLoop,
                       std::function<bool()> cancelRequested,
                       std::chrono::milliseconds loopTimeout) {
    BT::NodeStatus result = BT::NodeStatus::RUNNING;

    try {
        while (result == BT::NodeStatus::RUNNING) {
            if (cancelRequested()) {
                tree->haltTree();
                return BtStatus::CANCELED;
            }

            result = tree->tickOnce();
            onLoop();
            std::this_thread::sleep_for(loopTimeout);
        }
    } catch (const std::exception& ex) {
        AERROR << "Behavior tree threw exception: " << ex.what()
               << ". Exiting with failure.";
        return BtStatus::FAILED;
    }

    return (result == BT::NodeStatus::SUCCESS) ? BtStatus::SUCCEEDED
                                               : BtStatus::FAILED;
}

BT::Tree BtEngine::CreateTreeFromText(const std::string& xml_string,
                                      BT::Blackboard::Ptr blackboard) {
    return factory_.createTreeFromText(xml_string, blackboard);
}

BT::Tree BtEngine::CreateTreeFromFile(const std::string& file_path,
                                      BT::Blackboard::Ptr blackboard) {
    return factory_.createTreeFromFile(file_path, blackboard);
}

void BtEngine::HaltAllActions(BT::Tree& tree) {
    tree.haltTree();
}

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
