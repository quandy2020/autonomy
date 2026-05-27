/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>

#include "autonomy/tasks/behavior_tree/behavior_tree_context.hpp"
#include "autonomy/tasks/behavior_tree/behavior_tree_engine.hpp"
#include "behaviortree_cpp/behavior_tree.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

/**
 * @brief In-process BT executor (Nav2 BtActionServer without ROS actions).
 */
class BtActionServer
{
public:
    using OnGoalReceivedCallback = std::function<bool()>;
    using OnLoopCallback = std::function<void()>;
    using OnPreemptCallback = std::function<void()>;
    using OnCompletionCallback = std::function<void(BtStatus)>;

    BtActionServer(std::shared_ptr<BehaviorTreeEngine> engine,
                   std::shared_ptr<BehaviorTreeContext> ctx,
                   std::string default_bt_xml,
                   OnGoalReceivedCallback on_goal_received,
                   OnLoopCallback on_loop,
                   OnPreemptCallback on_preempt,
                   OnCompletionCallback on_completion);

    ~BtActionServer();

    bool LoadBehaviorTree(const std::string& bt_xml_filename = "");

    bool Start();
    bool Cancel();
    bool IsRunning() const;

    BT::Blackboard::Ptr GetBlackboard() const { return blackboard_; }

private:
    void ExecuteCallback();

    std::shared_ptr<BehaviorTreeEngine> engine_;
    std::shared_ptr<BehaviorTreeContext> ctx_;
    std::string default_bt_xml_;
    std::string current_bt_xml_;

    OnGoalReceivedCallback on_goal_received_;
    OnLoopCallback on_loop_;
    OnPreemptCallback on_preempt_;
    OnCompletionCallback on_completion_;

    BT::Blackboard::Ptr blackboard_;
    BT::Tree tree_;
    bool tree_loaded_{false};

    std::future<void> execution_future_;
    std::atomic<bool> stop_requested_{false};
    mutable std::mutex mutex_;
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
