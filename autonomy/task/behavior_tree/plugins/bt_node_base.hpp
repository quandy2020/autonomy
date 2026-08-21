/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/condition_node.h>

#include <functional>

#include "autonomy/task/behavior_tree/bt_node_registry.hpp"

// Nodes are linked into libautonomy.so; register via static initializers
// instead of exporting BT_RegisterNodesFromPlugin from separate plugin .so
// files.
#undef BT_REGISTER_NODES
#define BT_REGISTER_NODES(factory)                                             \
    static void AutonomyRegisterBtNodes(BT::BehaviorTreeFactory& factory);     \
    namespace {                                                                \
    struct AutonomyBtNodeRegistrar {                                           \
        AutonomyBtNodeRegistrar()                                              \
        {                                                                      \
            ::autonomy::task::AddBtNodeRegistrar(&AutonomyRegisterBtNodes);    \
        }                                                                      \
    };                                                                         \
    static const AutonomyBtNodeRegistrar autonomy_bt_node_registrar;           \
    }                                                                          \
    static void AutonomyRegisterBtNodes(BT::BehaviorTreeFactory& factory)

namespace autonomy {
namespace task {
namespace plugins {

/** Instant BT action (SUCCESS / FAILURE, no RUNNING). */
class BtSyncAction : public BT::SyncActionNode
{
public:
    using BT::SyncActionNode::SyncActionNode;

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override { return OnExecute(); }

protected:
    virtual BT::NodeStatus OnExecute() { return BT::NodeStatus::SUCCESS; }
};

/** Stateful BT action with onStart / onRunning / onHalted hooks. */
class BtStatefulAction : public BT::StatefulActionNode
{
public:
    using BT::StatefulActionNode::StatefulActionNode;

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus onStart() override
    {
        started_ = false;
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        if (!started_) {
            started_ = true;
            return OnFirstTick();
        }
        return OnExecute();
    }

    void onHalted() override { OnHalted(); }

    std::function<bool()> HaltCancelChecker() const
    {
        return [this]() { return isHaltRequested(); };
    }

protected:
    virtual BT::NodeStatus OnFirstTick() { return BT::NodeStatus::RUNNING; }

    virtual BT::NodeStatus OnExecute() { return BT::NodeStatus::SUCCESS; }

    virtual void OnHalted() {}

private:
    bool started_{false};
};

class BtCondition : public BT::ConditionNode
{
public:
    using BT::ConditionNode::ConditionNode;

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override { return OnEvaluate(); }

protected:
    virtual BT::NodeStatus OnEvaluate() { return BT::NodeStatus::SUCCESS; }
};

}  // namespace plugins
}  // namespace task
}  // namespace autonomy
