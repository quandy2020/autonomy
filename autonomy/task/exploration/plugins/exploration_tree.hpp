/* Copyright 2026 The Openbot Authors. */
#pragma once
#include <memory>
#include "autonomy/task/exploration/plugins/plugin_utils.hpp"

namespace autonomy::task::exploration {

// Tree success means a stage finished, not that the exploration task succeeded.
// The authoritative terminal result comes only from the session snapshot.
class ExplorationTree {
public:
    explicit ExplorationTree(std::shared_ptr<plugins::exploration::ExplorationBtContext> context);
    ~ExplorationTree();
    ExplorationTree(const ExplorationTree&) = delete;
    ExplorationTree& operator=(const ExplorationTree&) = delete;
    ExplorationSnapshot Tick();
    void Halt();
    BT::Blackboard::Ptr blackboard() const;
private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};
}  // namespace autonomy::task::exploration
