/*
 * Copyright 2026 The Openbot Authors
 */
/** @file bt_runner.cpp @brief Cooperative native tree execution and admission. */

#include "autonomy/task/behavior_tree/bt_runner.hpp"

#include <algorithm>
#include <chrono>
#include <exception>
#include <filesystem>
#include <string>
#include <thread>

#include "autonomy/common/logging.hpp"
#include "autonomy/common/nav_log.hpp"
#include "autonomy/task/behavior_tree/bt_node_registry.hpp"
#include "autonomy/task/behavior_tree/bt_status_logger.hpp"
#include "behaviortree_cpp/decorators/retry_node.h"

namespace autonomy {
namespace task {
namespace {

/** @brief Halt active nodes on their worker, keeping cleanup exceptions bounded. */
bool HaltTreeSafely(BT::Tree* tree)
{
    if (tree == nullptr || tree->rootNode() == nullptr) {
        return true;
    }
    try {
        tree->haltTree();
        return true;
    } catch (const std::exception& ex) {
        AERROR << "nav bt halt_exception err=" << ex.what();
    } catch (...) {
        AERROR << "nav bt halt_exception non_standard";
    }
    return false;
}

}  // namespace

thread_local const BtRunner* BtRunner::worker_context_ = nullptr;

BtRunner::~BtRunner() { StopWorker(); }

bool BtRunner::Configure(const BtProfile& profile)
{
    // Check before acquiring management: an external owner may already be
    // holding it while joining this very callback thread.
    if (worker_context_ == this) {
        return false;
    }
    std::lock_guard<std::mutex> lock(management_mutex_);
    StopWorker();
    try {
        profile_ = profile;
        factory_ = std::make_unique<BT::BehaviorTreeFactory>();
        RegisterBuiltinBtNodes(*factory_);
        // BT.CPP registers RetryUntilSuccessful; existing task XMLs use Retry.
        try {
            factory_->registerNodeType<BT::RetryNode>("Retry");
        } catch (const BT::BehaviorTreeException&) {
            // The native registry may already provide this compatibility id.
        }
        BT::ReactiveSequence::EnableException(false);
        BT::ReactiveFallback::EnableException(false);
        return true;
    } catch (const std::exception& ex) {
        AERROR << "nav bt configure_fail err=" << ex.what();
    } catch (...) {
        AERROR << "nav bt configure_fail non_standard";
    }
    factory_.reset();
    state_.store(BtRunState::kFailed);
    return false;
}

void BtRunner::SetHooks(BtRunnerHooks* hooks)
{
    if (worker_context_ == this) {
        return;
    }
    std::lock_guard<std::mutex> lock(management_mutex_);
    if (running_.load()) {
        AWARN << "nav bt reject_hooks reason=worker_active";
        return;
    }
    hooks_ = hooks;
}

bool BtRunner::Run(const std::string& tree_xml_path)
{
    if (worker_context_ == this) {
        return false;
    }
    std::lock_guard<std::mutex> lock(management_mutex_);
    if (tree_xml_path.empty()) {
        AERROR << "nav bt empty_path";
        return false;
    }
    if (!factory_) {
        AERROR << "nav bt not_configured";
        return false;
    }
    std::error_code error;
    if (!std::filesystem::is_regular_file(tree_xml_path, error)) {
        AERROR << "nav bt missing file=" << tree_xml_path;
        return false;
    }

    // The previous worker must finish halting its tree before its cancellation
    // flag, factory, hooks or active tree are reused for another goal.
    StopWorker();
    active_tree_ = tree_xml_path;
    cancel_requested_.store(false);
    paused_.store(false);
    state_.store(BtRunState::kRunning);
    running_.store(true);
    try {
        worker_ = std::thread([this]() { WorkerLoop(); });
    } catch (const std::exception& ex) {
        running_.store(false);
        state_.store(BtRunState::kFailed);
        AERROR << "nav bt worker_start_fail err=" << ex.what();
        return false;
    }
    AINFO << "nav bt tree=" << ::autonomy::common::NavBasename(active_tree_);
    return true;
}

bool BtRunner::Cancel()
{
    RequestCancellation();
    if (worker_context_ == this) {
        // The worker finishes halting and publishing before its owner joins.
        return true;
    }
    std::lock_guard<std::mutex> lock(management_mutex_);
    StopWorker();
    // A failed halt or callback remains a fault rather than a clean cancel.
    if (state_.load() != BtRunState::kFailed) {
        state_.store(BtRunState::kCanceled);
    }
    return true;
}

bool BtRunner::Pause()
{
    {
        std::lock_guard<std::mutex> wait_lock(wait_mutex_);
        paused_.store(true);
        ++wake_generation_;
    }
    wake_.notify_all();
    return true;
}

bool BtRunner::Resume()
{
    {
        std::lock_guard<std::mutex> wait_lock(wait_mutex_);
        paused_.store(false);
        ++wake_generation_;
    }
    wake_.notify_all();
    return true;
}

void BtRunner::WorkerLoop()
{
    const BtRunner* previous_context = worker_context_;
    worker_context_ = this;
    BtRunnerHooks* const hooks = hooks_;
    BT::Tree tree;
    std::unique_ptr<BtStatusLogger> status_logger;
    BtRunState outcome = BtRunState::kFailed;
    try {
        auto blackboard = BT::Blackboard::create();
        if (hooks != nullptr && !cancel_requested_.load()) {
            hooks->SetupBlackboard(blackboard);
        }
        if (cancel_requested_.load()) {
            outcome = BtRunState::kCanceled;
        } else {
            tree = CreateTreeFromFile(active_tree_, blackboard);
            if (hooks != nullptr && tree.rootNode() != nullptr) {
                status_logger = std::make_unique<BtStatusLogger>(tree.rootNode());
                status_logger->setFlushCallback(
                    [hooks](const std::vector<BtStatusEvent>& events) {
                        hooks->OnStatusLog(events);
                    });
            }
            const auto loop_period = std::chrono::milliseconds(
                std::max<uint32_t>(1, profile_.loop_period_ms));
            outcome = RunTree(&tree, hooks, loop_period, status_logger.get());
        }
    } catch (const std::exception& ex) {
        AERROR << "nav bt load_fail tree=" << ::autonomy::common::NavBasename(active_tree_)
               << " err=" << ex.what();
    } catch (...) {
        AERROR << "nav bt worker_exception non_standard";
    }
    if (cancel_requested_.load() && outcome != BtRunState::kFailed) {
        outcome = BtRunState::kCanceled;
    }
    if (!HaltTreeSafely(&tree)) {
        outcome = BtRunState::kFailed;
    }
    state_.store(outcome);
    try {
        if (status_logger) {
            status_logger->flush();
        }
        if (hooks != nullptr) {
            hooks->OnTick();
        }
    } catch (const std::exception& ex) {
        AERROR << "nav bt terminal_callback_exception err=" << ex.what();
        state_.store(BtRunState::kFailed);
    } catch (...) {
        AERROR << "nav bt terminal_callback_exception non_standard";
        state_.store(BtRunState::kFailed);
    }
    // IsRunning=false means no host callbacks or tree destruction remain.
    status_logger.reset();
    tree = BT::Tree();
    worker_context_ = previous_context;
    running_.store(false);
}

void BtRunner::RequestCancellation()
{
    {
        std::lock_guard<std::mutex> wait_lock(wait_mutex_);
        cancel_requested_.store(true);
    }
    wake_.notify_all();
}

void BtRunner::StopWorker()
{
    RequestCancellation();
    if (worker_.joinable()) {
        worker_.join();
    }
    running_.store(false);
}

BT::Tree BtRunner::CreateTreeFromFile(const std::string& file_path,
                                      BT::Blackboard::Ptr blackboard)
{
    return factory_->createTreeFromFile(file_path, blackboard);
}

BtRunState BtRunner::RunTree(BT::Tree* tree, BtRunnerHooks* hooks,
                             std::chrono::milliseconds loop_period,
                             BtStatusLogger* status_logger)
{
    BT::NodeStatus result = BT::NodeStatus::RUNNING;

    try {
        while (result == BT::NodeStatus::RUNNING) {
            uint64_t observed_wake;
            {
                std::lock_guard<std::mutex> wait_lock(wait_mutex_);
                observed_wake = wake_generation_;
            }
            if (cancel_requested_.load()) {
                return BtRunState::kCanceled;
            }

            if (!paused_.load()) {
                if (hooks != nullptr && !hooks->BeforeTick()) {
                    return BtRunState::kCanceled;
                }
                // A host check may invoke callbacks that cancel or pause. The
                // check is not permission to execute after such a revocation.
                if (cancel_requested_.load()) {
                    return BtRunState::kCanceled;
                }
                if (!paused_.load()) {
                    result = tree->tickOnce();
                    if (result != BT::NodeStatus::RUNNING)
                        AINFO << "nav bt tick_result="
                              << (result == BT::NodeStatus::SUCCESS ? "SUCCESS" : "FAILURE");
                }
            }
            if (status_logger != nullptr) {
                status_logger->flush();
            }
            if (hooks != nullptr) {
                hooks->OnTick();
            }
            if (cancel_requested_.load()) {
                return BtRunState::kCanceled;
            }
            if (result == BT::NodeStatus::RUNNING) {
                std::unique_lock<std::mutex> wait_lock(wait_mutex_);
                wake_.wait_for(wait_lock, loop_period,
                    [this, observed_wake] {
                        return cancel_requested_.load() ||
                               wake_generation_ != observed_wake;
                    });
            }
        }
    } catch (const std::exception& ex) {
        AERROR << "nav bt exception err=" << ex.what();
        return BtRunState::kFailed;
    } catch (...) {
        AERROR << "nav bt exception non_standard";
        return BtRunState::kFailed;
    }

    return (result == BT::NodeStatus::SUCCESS) ? BtRunState::kSucceeded
                                               : BtRunState::kFailed;
}

}  // namespace task
}  // namespace autonomy
