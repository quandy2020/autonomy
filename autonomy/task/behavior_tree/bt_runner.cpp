/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/behavior_tree/bt_runner.hpp"

#include <chrono>
#include <filesystem>
#include <string>
#include <thread>

#include "autonomy/common/logging.hpp"
#include "autonomy/task/behavior_tree/bt_node_registry.hpp"
#include "autonomy/task/behavior_tree/bt_status_logger.hpp"
#include "behaviortree_cpp/decorators/retry_node.h"

namespace autonomy {
namespace task {

BtRunner::~BtRunner() { StopWorker(); }

bool BtRunner::Configure(const BtProfile& profile)
{
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
        AERROR << "BtRunner: empty behavior tree path";
        return false;
    }
    if (!factory_) {
        AERROR << "BtRunner: not configured";
        return false;
    }
    if (!std::filesystem::exists(tree_xml_path)) {
        AERROR << "BtRunner: behavior tree file not found: " << tree_xml_path;
        return false;
    }

    DetachWorker();
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
    if (hooks_ != nullptr) {
        hooks_->SetupBlackboard(blackboard);
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
    running_.store(false);
}

void BtRunner::DetachWorker()
{
    cancel_requested_.store(true);
    running_.store(false);
    if (worker_.joinable()) {
        worker_.detach();
    }
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
        AERROR << "BtRunner: tree exception: " << ex.what();
        return BtRunState::kFailed;
    }

    return (result == BT::NodeStatus::SUCCESS) ? BtRunState::kSucceeded
                                               : BtRunState::kFailed;
}

}  // namespace task
}  // namespace autonomy
