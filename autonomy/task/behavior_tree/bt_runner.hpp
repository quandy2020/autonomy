/*
 * Copyright 2026 The Openbot Authors
 *
 * Builds trees and runs tick loops on a worker thread.
 */
/**
 * @file bt_runner.hpp
 * @brief Native BT host with cooperative cancellation and pre-tick admission.
 */

#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "autonomy/task/behavior_tree/bt_profile.hpp"
#include "autonomy/task/behavior_tree/bt_status_logger.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_cpp/bt_factory.h"

namespace autonomy {
namespace task {

/** @brief Outcome of the current native tree invocation. */
enum class BtRunState {
    kIdle,
    kRunning,
    kSucceeded,
    kFailed,
    kCanceled,
};

/**
 * Non-template hooks for BtRunner. Prefer this over std::function+lambda in
 * template hosts (GCC 11 ICE at -O3 on std::_Function_handler).
 */
class BtRunnerHooks {
public:
    /// Hooks are nonowning; their host outlives the worker and its final callback.
    virtual ~BtRunnerHooks() = default;
    /// Populate a new worker-owned blackboard before constructing the tree.
    virtual void SetupBlackboard(const BT::Blackboard::Ptr& /*blackboard*/) {}
    /**
     * @brief Authorize the next unpaused tree tick before any node executes.
     * @return False to halt this invocation with kCanceled, without ticking it.
     *
     * The default preserves tasks with no external movement lease. Domain
     * hosts check their revocable lease here; actual remote dispatch must also
     * check that lease. Return promptly and do not restart/configure the runner
     * from a hook. Cancel/Pause/Resume may be requested from worker callbacks.
     * Cancellation and pause are checked again after this hook returns.
     */
    virtual bool BeforeTick() { return true; }
    /// Publish monitoring after each iteration and after the terminal state.
    virtual void OnTick() {}
    /// Receive buffered transitions outside runner management locks.
    virtual void OnStatusLog(const std::vector<BtStatusEvent>& /*events*/) {}
};

/** Builds trees and runs tick loops on a worker thread. */
class BtRunner
{
public:
    /// Construct an idle runner with no configured factory or callback host.
    BtRunner() = default;
    /// Cancel and join on an external owner; destruction from hooks is invalid.
    ~BtRunner();

    BtRunner(const BtRunner&) = delete;
    BtRunner& operator=(const BtRunner&) = delete;

    /** @brief Stop the previous worker and configure native node registration. */
    bool Configure(const BtProfile& profile);
    /** @brief Install a nonowning host while idle; reject changes during a run. */
    void SetHooks(BtRunnerHooks* hooks);

    /** @brief Stop the preceding invocation and launch an existing XML file. */
    bool Run(const std::string& tree_xml_path);
    /**
     * @brief Revoke ticks immediately and join from an external caller.
     * Worker callbacks only request cancellation; their external owner joins.
     */
    bool Cancel();
    /// Skip ticks while continuing post-tick monitoring.
    bool Pause();
    /// Allow subsequent ticks after a fresh BeforeTick admission check.
    bool Resume();

    /// True while any worker cleanup or host callback remains in flight.
    bool IsRunning() const { return running_.load(); }
    /// True only inside this runner's callbacks; reentrant starts must refuse.
    bool IsWorkerThread() const { return worker_context_ == this; }
    /// Current invocation outcome; final once IsRunning becomes false.
    BtRunState state() const { return state_.load(); }
    /// Caller serializes this pathname read against configure/run operations.
    const std::string& active_tree() const { return active_tree_; }

private:
    /// Wake timed waits without losing a cancellation at the wait boundary.
    void RequestCancellation();
    /// Build, run, halt and publish one invocation entirely on its worker.
    void WorkerLoop();
    /// Stop and join while management is serialized; never called from a hook.
    void StopWorker();
    /// Build a tree using the current immutable factory configuration.
    BT::Tree CreateTreeFromFile(const std::string& file_path,
                                BT::Blackboard::Ptr blackboard);
    /// Admit every tick and convert node/hook exceptions to a failed outcome.
    BtRunState RunTree(BT::Tree* tree, BtRunnerHooks* hooks,
                       std::chrono::milliseconds loop_period,
                       BtStatusLogger* status_logger);

    /// Configuration is only replaced after the previous worker has joined.
    BtProfile profile_;
    /// Native registry shared read-only by the active invocation.
    std::unique_ptr<BT::BehaviorTreeFactory> factory_;
    /// Nonowning host, installed while no callbacks remain in flight.
    BtRunnerHooks* hooks_ = nullptr;
    /// XML pathname of the invocation; unchanged until its worker has joined.
    std::string active_tree_;
    /// Terminal/running outcome published before the final monitoring callback.
    std::atomic<BtRunState> state_{BtRunState::kIdle};
    /// True until all worker callbacks have completed, including cleanup.
    std::atomic<bool> running_{false};
    /// Pauses tree advancement without suspending health monitoring.
    std::atomic<bool> paused_{false};
    /// Immediate cooperative cancellation, independent of management joins.
    std::atomic<bool> cancel_requested_{false};
    /// Join ownership belongs to serialized external management callers.
    std::thread worker_;
    /// Serializes external configure/run/hook replacement and worker joins.
    std::mutex management_mutex_;
    /// Protects timed waiting only; no host callbacks run with this lock held.
    std::mutex wait_mutex_;
    /// Changes under wait_mutex_ so pause/resume notifications cannot be lost.
    uint64_t wake_generation_{0};
    /// Makes lifecycle changes interrupt even a long configured loop period.
    std::condition_variable wake_;
    /// Identifies callback reentry without racing access to the thread object.
    static thread_local const BtRunner* worker_context_;
};

}  // namespace task
}  // namespace autonomy
