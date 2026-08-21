/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/behavior_tree/bt_node_registry.hpp"

#include <mutex>
#include <string>
#include <vector>

namespace autonomy {
namespace task {
namespace {

std::mutex& RegistrarMutex()
{
    static std::mutex mutex;
    return mutex;
}

std::vector<BtNodeRegistrar>& Registrars()
{
    static std::vector<BtNodeRegistrar> registrars;
    return registrars;
}

}  // namespace

void AddBtNodeRegistrar(BtNodeRegistrar registrar)
{
    if (registrar == nullptr) {
        return;
    }
    std::lock_guard<std::mutex> lock(RegistrarMutex());
    Registrars().push_back(registrar);
}

void RegisterBuiltinBtNodes(BT::BehaviorTreeFactory& factory)
{
    std::lock_guard<std::mutex> lock(RegistrarMutex());
    for (BtNodeRegistrar registrar : Registrars()) {
        try {
            registrar(factory);
        } catch (const BT::BehaviorTreeException& ex) {
            // Shared XML ids (e.g. ClearCostmap) must register once; skip dupes.
            if (std::string(ex.what()).find("already registered") ==
                std::string::npos) {
                throw;
            }
        }
    }
}

}  // namespace task
}  // namespace autonomy
