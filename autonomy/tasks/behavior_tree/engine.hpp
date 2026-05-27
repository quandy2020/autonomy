/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "autonomy/tasks/proto/task_options.pb.h"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

enum class RunStatus { SUCCEEDED, FAILED, CANCELED };

class BehaviorTreeEngine
{
public:
    explicit BehaviorTreeEngine(const proto::TaskOptions& options);
    ~BehaviorTreeEngine() = default;

    RunStatus Run(BT::Tree* tree, std::function<void()> on_loop,
                 std::function<bool()> cancel_requested,
                 std::chrono::milliseconds loop_timeout =
                     std::chrono::milliseconds(10));

    BT::Tree CreateTreeFromFile(const std::string& file_path,
                                BT::Blackboard::Ptr blackboard);

    void HaltAllActions(BT::Tree& tree);

    BT::BehaviorTreeFactory& Factory() { return factory_; }

private:
    void RegisterPlugins(const proto::TaskOptions& options);

    BT::BehaviorTreeFactory factory_;
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
