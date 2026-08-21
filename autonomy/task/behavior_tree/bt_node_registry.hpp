/*
 * Copyright 2026 The Openbot Authors
 *
 * Static BT node registration for nodes linked into libautonomy.so.
 */

#pragma once

#include "behaviortree_cpp/bt_factory.h"

namespace autonomy {
namespace task {

using BtNodeRegistrar = void (*)(BT::BehaviorTreeFactory&);

/** Append a registrar invoked by RegisterBuiltinBtNodes. */
void AddBtNodeRegistrar(BtNodeRegistrar registrar);

/** Register every BT node compiled into libautonomy. */
void RegisterBuiltinBtNodes(BT::BehaviorTreeFactory& factory);

}  // namespace task
}  // namespace autonomy
