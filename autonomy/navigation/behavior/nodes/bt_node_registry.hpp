/*
 * Copyright 2026 The Navigation Authors (navigation)
 */

/**
 * @file bt_node_registry.hpp
 * @brief Static registration for navigation BehaviorTree.CPP nodes.
 */

#pragma once

#include "behaviortree_cpp/bt_factory.h"

namespace navigation
{
namespace behavior
{

/** @brief Function invoked with the factory to register one node type. */
using BtNodeRegistrar = void (*)(BT::BehaviorTreeFactory&);

/** @brief Append a node registrar. */
void AddBtNodeRegistrar(BtNodeRegistrar registrar);

/** @brief Register all navigation nodes compiled into the library. */
void RegisterBuiltinBtNodes(BT::BehaviorTreeFactory& factory);

}  // namespace behavior
}  // namespace navigation
