/*
 * Copyright 2026 The Navigation Authors (navigation)
 */

#include "navigation/behavior/nodes/bt_node_registry.hpp"

#include <mutex>
#include <string>
#include <vector>

namespace navigation
{
namespace behavior
{
namespace
{

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
    if (registrar == nullptr)
    {
        return;
    }
    const std::lock_guard<std::mutex> lock(RegistrarMutex());
    Registrars().push_back(registrar);
}

void RegisterBuiltinBtNodes(BT::BehaviorTreeFactory& factory)
{
    const std::lock_guard<std::mutex> lock(RegistrarMutex());
    for (BtNodeRegistrar registrar : Registrars())
    {
        try
        {
            registrar(factory);
        }
        catch (const BT::BehaviorTreeException& ex)
        {
            if (std::string(ex.what()).find("already registered") ==
                std::string::npos)
            {
                throw;
            }
        }
    }
}

}  // namespace behavior
}  // namespace navigation
