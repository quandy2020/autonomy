/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/tasks/behavior_tree/engine.hpp"

#include <cstdlib>
#include <filesystem>

#include "autonomy/common/logging.hpp"
#include "autonomy/tasks/behavior_tree/utils.hpp"
#include "autonomy/tasks/behavior_tree/loop_rate.hpp"
#include "behaviortree_cpp/utils/shared_library.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

BehaviorTreeEngine::BehaviorTreeEngine(const proto::TaskOptions& options) {
    BT::ReactiveSequence::EnableException(false);
    BT::ReactiveFallback::EnableException(false);
    RegisterPlugins(options);
}

void BehaviorTreeEngine::RegisterPlugins(const proto::TaskOptions& options) {
    const auto search_dirs = ResolvePluginLibraryPaths(options);

    std::string ld_library_path;
    for (const auto& dir : search_dirs) {
        if (!ld_library_path.empty()) {
            ld_library_path += ":";
        }
        ld_library_path += dir;
    }
    if (const char* existing = std::getenv("LD_LIBRARY_PATH");
        existing != nullptr && existing[0] != '\0') {
        if (!ld_library_path.empty()) {
            ld_library_path += ":";
        }
        ld_library_path += existing;
    }
    if (!ld_library_path.empty()) {
        setenv("LD_LIBRARY_PATH", ld_library_path.c_str(), 1);
    }

    BT::SharedLibrary loader;

    for (const auto& lib_name : options.plugin_lib_names()) {
        bool loaded = false;
        for (const auto& dir : search_dirs) {
            const std::string path = dir + "/lib" + lib_name + ".so";
            const std::string alt = dir + "/" + lib_name + ".so";
            if (!std::filesystem::exists(path) && !std::filesystem::exists(alt)) {
                continue;
            }
            try {
                factory_.registerFromPlugin(loader.getOSName(lib_name));
                loaded = true;
                break;
            } catch (const std::exception& ex) {
                AWARN << "Failed to load BT plugin " << lib_name << " from "
                      << dir << ": " << ex.what();
            }
        }
        if (!loaded) {
            try {
                factory_.registerFromPlugin(loader.getOSName(lib_name));
                loaded = true;
            } catch (const std::exception& ex) {
                AWARN << "Failed to load BT plugin " << lib_name << ": " << ex.what();
            }
        }
        if (!loaded) {
            AERROR << "BT plugin not loaded: " << lib_name;
        }
    }
}

RunStatus BehaviorTreeEngine::Run(BT::Tree* tree, std::function<void()> on_loop,
                                 std::function<bool()> cancel_requested,
                                 std::chrono::milliseconds loop_timeout) {
    if (!tree) {
        return RunStatus::FAILED;
    }
    LoopRate loop_rate(loop_timeout, tree);
    BT::NodeStatus result = BT::NodeStatus::RUNNING;

    try {
        while (result == BT::NodeStatus::RUNNING) {
            if (cancel_requested && cancel_requested()) {
                tree->haltTree();
                return RunStatus::CANCELED;
            }
            result = tree->tickOnce();
            if (on_loop) {
                on_loop();
            }
            loop_rate.sleep();
        }
    } catch (const std::exception& ex) {
        AERROR << "Behavior tree exception: " << ex.what();
        return RunStatus::FAILED;
    }

    return (result == BT::NodeStatus::SUCCESS) ? RunStatus::SUCCEEDED
                                               : RunStatus::FAILED;
}

BT::Tree BehaviorTreeEngine::CreateTreeFromFile(
    const std::string& file_path, BT::Blackboard::Ptr blackboard) {
    return factory_.createTreeFromFile(file_path, blackboard);
}

void BehaviorTreeEngine::HaltAllActions(BT::Tree& tree) {
    tree.haltTree();
}

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
