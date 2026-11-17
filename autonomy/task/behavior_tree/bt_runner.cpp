/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/behavior_tree/bt_runner.hpp"

#include <chrono>
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

BtRunner::~BtRunner() { StopWorker(); }

bool BtRunner::Configure(const BtProfile& profile)
{
    std::lock_guard<std::mutex> lock(management_mutex_);
    if (worker_.joinable() && worker_.get_id() == std::this_thread::get_id()) {
        return false;
    }
    StopWorker();
    profile_ = profile;
    factory_ = std::make_unique<BT::BehaviorTreeFactory>();

    // BT nodes are compiled into libautonomy.so and registered statically.
    RegisterBuiltinBtNodes(*factory_);

    // Alias used by some task XMLs / docs (BT.CPP registers RetryUntilSuccessful).
    try {
        factory_->registerNodeType<BT::RetryNode>("Retry");
    } catch (const BT::BehaviorTreeException&) {
        // already present
    }

    BT::ReactiveSequence::EnableException(false);
    BT::ReactiveFallback::EnableException(false);
    return true;
}

void BtRunner::SetHooks(BtRunnerHooks* hooks) { hooks_ = hooks; }

bool BtRunner::Run(const std::string& tree_xml_path)
{
    if (tree_xml_path.empty()) {
        AERROR << "nav bt empty_path";
        return false;
    }
    if (!factory_) {
        AERROR << "nav bt not_configured";
        return false;
    }
    if (!std::filesystem::exists(tree_xml_path)) {
        AERROR << "nav bt missing file=" << tree_xml_path;
        return false;
    }

    std::lock_guard<std::mutex> lock(management_mutex_);
    if (worker_.joinable() && worker_.get_id() == std::this_thread::get_id()) {
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
    worker_ = std::thread([this]() { WorkerLoop(); });
    AINFO << "nav bt tree=" << ::autonomy::common::NavBasename(active_tree_);
    return true;
}

bool BtRunner::Cancel()
{
    std::lock_guard<std::mutex> lock(management_mutex_);
    cancel_requested_.store(true);
    StopWorker();
    state_.store(BtRunState::kCanceled);
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
    if (hooks_ != nullptr) {
        hooks_->SetupBlackboard(blackboard);
    }

    BT::Tree tree;
    try {
        tree = CreateTreeFromFile(active_tree_, blackboard);
    } catch (const std::exception& ex) {
        AERROR << "nav bt load_fail tree=" << ::autonomy::common::NavBasename(active_tree_)
               << " err=" << ex.what();
        state_.store(BtRunState::kFailed);
        if (hooks_ != nullptr) {
            hooks_->OnTick();
        }
        running_.store(false);
        return;
    }

    std::unique_ptr<BtStatusLogger> status_logger;
    if (hooks_ != nullptr && tree.rootNode() != nullptr) {
        status_logger = std::make_unique<BtStatusLogger>(tree.rootNode());
        status_logger->setFlushCallback(
            [hooks = hooks_](const std::vector<BtStatusEvent>& events) {
                if (hooks != nullptr) {
                    hooks->OnStatusLog(events);
                }
            });
    }

    const auto loop_period =
        std::chrono::milliseconds(profile_.loop_period_ms);
    state_.store(RunTree(&tree, hooks_, loop_period, status_logger.get()));
    if (status_logger) {
        status_logger->flush();
    }
    if (hooks_ != nullptr) {
        hooks_->OnTick();
    }
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

BtRunState BtRunner::RunTree(BT::Tree* tree, BtRunnerHooks* hooks,
                             std::chrono::milliseconds loop_period,
                             BtStatusLogger* status_logger)
{
    BT::NodeStatus result = BT::NodeStatus::RUNNING;

    try {
        while (result == BT::NodeStatus::RUNNING) {
            if (cancel_requested_.load()) {
                tree->haltTree();
                if (status_logger != nullptr) {
                    status_logger->flush();
                }
                return BtRunState::kCanceled;
            }

            if (!paused_.load()) {
                result = tree->tickOnce();
            }
            if (status_logger != nullptr) {
                status_logger->flush();
            }
            if (hooks != nullptr) {
                hooks->OnTick();
            }
            std::this_thread::sleep_for(loop_period);
        }
    } catch (const std::exception& ex) {
        AERROR << "nav bt exception err=" << ex.what();
        return BtRunState::kFailed;
    }

    return (result == BT::NodeStatus::SUCCESS) ? BtRunState::kSucceeded
                                               : BtRunState::kFailed;
}

}  // namespace task
}  // namespace autonomy
