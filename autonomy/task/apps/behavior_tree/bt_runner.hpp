/*
 * Copyright 2026 The Openbot Authors
 *
 * Loads BT plugins, builds trees, and runs tick loops on a worker thread.
 */

#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "autonomy/task/apps/behavior_tree/bt_profile.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_cpp/bt_factory.h"

namespace autonomy {
namespace task {

enum class BtRunState {
    kIdle,
    kRunning,
    kSucceeded,
    kFailed,
    kCanceled,
};

/** Loads plugins and drives a behavior tree on a background thread. */
class BtRunner
{
public:
    using TickCallback = std::function<void()>;
    using BlackboardSetupCallback =
        std::function<void(const BT::Blackboard::Ptr&)>;

    BtRunner() = default;
    ~BtRunner();

    BtRunner(const BtRunner&) = delete;
    BtRunner& operator=(const BtRunner&) = delete;

    bool Configure(const BtProfile& profile);
    void SetBlackboardSetup(BlackboardSetupCallback callback);
    void SetTickCallback(TickCallback callback);

    bool Run(const std::string& tree_xml_path);
    bool Cancel();
    bool Pause();
    bool Resume();

    bool IsRunning() const { return running_.load(); }
    BtRunState state() const { return state_.load(); }
    const std::string& active_tree() const { return active_tree_; }

private:
    void WorkerLoop();
    void StopWorker();
    BT::Tree CreateTreeFromFile(const std::string& file_path,
                                BT::Blackboard::Ptr blackboard);
    BtRunState RunTree(BT::Tree* tree, const std::function<void()>& on_tick,
                       std::chrono::milliseconds loop_period);

    BtProfile profile_;
    std::unique_ptr<BT::BehaviorTreeFactory> factory_;
    BlackboardSetupCallback blackboard_setup_;
    TickCallback tick_callback_;
    std::string active_tree_;
    std::atomic<BtRunState> state_{BtRunState::kIdle};
    std::atomic<bool> running_{false};
    std::atomic<bool> paused_{false};
    std::atomic<bool> cancel_requested_{false};
    std::thread worker_;
};

}  // namespace task
}  // namespace autonomy
