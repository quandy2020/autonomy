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

namespace autonomy {
namespace task {

BtRunner::~BtRunner() { StopWorker(); }

bool BtRunner::Configure(const BtProfile& profile)
{
    profile_ = profile;
    factory_ = std::make_unique<BT::BehaviorTreeFactory>();

    // BT nodes are compiled into libautonomy.so and registered statically.
    RegisterBuiltinBtNodes(*factory_);

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
    if (!std::filesystem::exists(tree_xml_path)) {
        AERROR << "BtRunner: behavior tree file not found: " << tree_xml_path;
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
