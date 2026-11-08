/*
 * Copyright 2026 The Navigation Authors (navigation)
 */

/**
 * @file bt_node_base.hpp
 * @brief BehaviorTree.CPP base classes for navigation nodes.
 */

#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/condition_node.h>

#include <functional>

#include "navigation/behavior/engine/blackboard_keys.hpp"
#include "navigation/behavior/nodes/bt_node_registry.hpp"
#include "navigation/behavior/nodes/node_context.hpp"

#undef BT_REGISTER_NODES
#define BT_REGISTER_NODES(factory)                                             \
    static void RegisterNavigationBtNodes(BT::BehaviorTreeFactory& factory);   \
    namespace {                                                                \
    struct NavigationBtRegistrar {                                             \
        NavigationBtRegistrar()                                                \
        {                                                                      \
            ::navigation::behavior::AddBtNodeRegistrar(                        \
                &RegisterNavigationBtNodes);                                   \
        }                                                                      \
    };                                                                         \
    static const NavigationBtRegistrar navigation_bt_registrar;                \
    }                                                                          \
    static void RegisterNavigationBtNodes(BT::BehaviorTreeFactory& factory)

namespace navigation
{
namespace behavior
{

/** @brief Immediate action returning SUCCESS/FAILURE. */
class NavigationSyncAction : public BT::SyncActionNode
{
public:
    using BT::SyncActionNode::SyncActionNode;

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override
    {
        NodeContext* context = Context();
        return context != nullptr ? OnExecute(*context) : BT::NodeStatus::FAILURE;
    }

protected:
    virtual BT::NodeStatus OnExecute(NodeContext& context) = 0;

    NodeContext* Context() const
    {
        NodeContext* context = nullptr;
        if (config().blackboard != nullptr &&
            config().blackboard->get(keys::kContext, context))
        {
            return context;
        }
        return nullptr;
    }
};


/** @brief Stateful BT action with OnFirstTick/OnExecute/OnHalted hooks. */
class NavigationStatefulAction : public BT::StatefulActionNode
{
public:
    using BT::StatefulActionNode::StatefulActionNode;

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus onStart() override
    {
        started_ = false;
        NodeContext* context = Context();
        if (context == nullptr)
        {
            return BT::NodeStatus::FAILURE;
        }
        return OnFirstTick(*context);
    }

    BT::NodeStatus onRunning() override
    {
        NodeContext* context = Context();
        if (context == nullptr)
        {
            return BT::NodeStatus::FAILURE;
        }
        if (!started_)
        {
            const BT::NodeStatus status = OnFirstTick(*context);
            if (status != BT::NodeStatus::RUNNING || LatchAfterFirstTick())
            {
                started_ = true;
            }
            return status;
        }
        return OnExecute(*context);
    }

    void onHalted() override
    {
        NodeContext* context = Context();
        if (context != nullptr)
        {
            OnHalted(*context);
        }
    }

protected:
    virtual BT::NodeStatus OnFirstTick(NodeContext& context)
    {
        (void)context;
        return BT::NodeStatus::RUNNING;
    }

    virtual BT::NodeStatus OnExecute(NodeContext& context) = 0;

    virtual void OnHalted(NodeContext& context) { (void)context; }

    virtual bool LatchAfterFirstTick() const { return true; }

    NodeContext* Context() const
    {
        NodeContext* context = nullptr;
        if (config().blackboard != nullptr &&
            config().blackboard->get(keys::kContext, context))
        {
            return context;
        }
        return nullptr;
    }

private:
    bool started_{false};
};

/** @brief Condition node returning SUCCESS/FAILURE. */
class NavigationCondition : public BT::ConditionNode
{
public:
    using BT::ConditionNode::ConditionNode;

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override
    {
        NodeContext* context = Context();
        return context != nullptr ? OnEvaluate(*context) : BT::NodeStatus::FAILURE;
    }

protected:
    virtual BT::NodeStatus OnEvaluate(const NodeContext& context) = 0;

    NodeContext* Context() const
    {
        NodeContext* context = nullptr;
        if (config().blackboard != nullptr &&
            config().blackboard->get(keys::kContext, context))
        {
            return context;
        }
        return nullptr;
    }
};

}  // namespace behavior
}  // namespace navigation
