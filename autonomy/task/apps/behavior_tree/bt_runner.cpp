/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/apps/behavior_tree/bt_runner.hpp"

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include "autonomy/common/config.hpp"
#include "autonomy/common/logging.hpp"
#include "behaviortree_cpp/utils/shared_library.h"

namespace autonomy {
namespace task {
namespace {

std::string PluginFilename(const std::string& plugin_name)
{
    BT::SharedLibrary loader;
    return loader.getOSName(plugin_name);
}

std::vector<std::filesystem::path> PluginSearchPaths(
    const std::string& plugin_lib_path)
{
    std::vector<std::filesystem::path> dirs;
    if (const char* env = std::getenv("AUTONOMY_BT_PLUGIN_PATH");
        env != nullptr && env[0] != '\0') {
        dirs.emplace_back(env);
    }
    if (!plugin_lib_path.empty()) {
        dirs.emplace_back(plugin_lib_path);
    }
    dirs.emplace_back(std::string(autonomy::common::kLibraryInstallDir) + "/lib");
    dirs.emplace_back(std::string(autonomy::common::kLibraryBuildDir) + "/lib");
    return dirs;
}

bool IsLoadablePlugin(const std::filesystem::path& path)
{
    std::error_code ec;
    if (!std::filesystem::is_regular_file(path, ec) || ec) {
        return false;
    }
    std::ifstream in(path, std::ios::binary);
    return in.good();
}

std::string ResolvePluginPath(const std::string& plugin_name,
                              const std::string& plugin_lib_path)
{
    const std::string filename = PluginFilename(plugin_name);
    for (const auto& dir : PluginSearchPaths(plugin_lib_path)) {
        const auto candidate = dir / filename;
        if (IsLoadablePlugin(candidate)) {
            return candidate.string();
        }
    }
    return filename;
}

}  // namespace

BtRunner::~BtRunner() { StopWorker(); }

bool BtRunner::Configure(const BtProfile& profile)
{
    profile_ = profile;
    factory_ = std::make_unique<BT::BehaviorTreeFactory>();

    for (const auto& plugin_name : profile_.plugin_libraries) {
        const std::string lib_path =
            ResolvePluginPath(plugin_name, profile_.plugin_lib_path);
        try {
            factory_->registerFromPlugin(lib_path);
        } catch (const std::exception& ex) {
            AERROR << "BtRunner: failed to load plugin '" << plugin_name
                   << "' from '" << lib_path << "': " << ex.what();
            factory_.reset();
            return false;
        }
    }

    BT::ReactiveSequence::EnableException(false);
    BT::ReactiveFallback::EnableException(false);
    return true;
}

void BtRunner::SetBlackboardSetup(BlackboardSetupCallback callback)
{
    blackboard_setup_ = std::move(callback);
}

void BtRunner::SetTickCallback(TickCallback callback)
{
    tick_callback_ = std::move(callback);
}

bool BtRunner::Run(const std::string& tree_xml_path)
{
    if (tree_xml_path.empty()) {
        AERROR << "BtRunner: empty behavior tree path";
        return false;
    }
    if (!factory_) {
        AERROR << "BtRunner: not configured";
        return false;
    }

    StopWorker();
    active_tree_ = tree_xml_path;
    cancel_requested_.store(false);
    paused_.store(false);
    state_.store(BtRunState::kRunning);
    running_.store(true);
    worker_ = std::thread([this]() { WorkerLoop(); });
    AINFO << "BtRunner: running " << active_tree_;
    return true;
}

bool BtRunner::Cancel()
{
    cancel_requested_.store(true);
    state_.store(BtRunState::kCanceled);
    StopWorker();
    return true;
}

bool BtRunner::Pause()
{
    paused_.store(true);
    return true;
}

bool BtRunner::Resume()
{
    paused_.store(false);
    return true;
}

void BtRunner::WorkerLoop()
{
    auto blackboard = BT::Blackboard::create();
    if (blackboard_setup_) {
        blackboard_setup_(blackboard);
    }

    BT::Tree tree;
    try {
        tree = CreateTreeFromFile(active_tree_, blackboard);
    } catch (const std::exception& ex) {
        AERROR << "BtRunner: failed to load tree " << active_tree_ << ": "
               << ex.what();
        state_.store(BtRunState::kFailed);
        running_.store(false);
        return;
    }

    const auto loop_period =
        std::chrono::milliseconds(profile_.loop_period_ms);
    state_.store(RunTree(
        &tree,
        [this]() {
            if (tick_callback_) {
                tick_callback_();
            }
        },
        loop_period));
    running_.store(false);
}

void BtRunner::StopWorker()
{
    cancel_requested_.store(true);
    running_.store(false);
    if (worker_.joinable()) {
        worker_.join();
    }
}

BT::Tree BtRunner::CreateTreeFromFile(const std::string& file_path,
                                      BT::Blackboard::Ptr blackboard)
{
    return factory_->createTreeFromFile(file_path, blackboard);
}

BtRunState BtRunner::RunTree(BT::Tree* tree,
                             const std::function<void()>& on_tick,
                             std::chrono::milliseconds loop_period)
{
    BT::NodeStatus result = BT::NodeStatus::RUNNING;

    try {
        while (result == BT::NodeStatus::RUNNING) {
            if (cancel_requested_.load()) {
                tree->haltTree();
                return BtRunState::kCanceled;
            }

            if (!paused_.load()) {
                result = tree->tickOnce();
            }
            if (on_tick) {
                on_tick();
            }
            std::this_thread::sleep_for(loop_period);
        }
    } catch (const std::exception& ex) {
        AERROR << "BtRunner: tree exception: " << ex.what();
        return BtRunState::kFailed;
    }

    return (result == BT::NodeStatus::SUCCESS) ? BtRunState::kSucceeded
                                               : BtRunState::kFailed;
}

}  // namespace task
}  // namespace autonomy
