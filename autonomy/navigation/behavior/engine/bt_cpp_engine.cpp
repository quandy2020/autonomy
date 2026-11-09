/* Copyright 2026 The Navigation Authors (navigation) */
#include "navigation/behavior/engine/bt_cpp_engine.hpp"

#include <filesystem>
#include "navigation/behavior/engine/blackboard_keys.hpp"
#include "navigation/behavior/nodes/bt_node_registry.hpp"
#include "navigation/common/configuration_loader.hpp"

namespace navigation::behavior {
bool BtCppEngine::Configure(const config::BehaviorOptions& options,
                            planning::NavigationSession* session) {
    Reset();
    tree_.reset();
    error_.clear();
    if (!session) {
        error_ = "missing navigation session";
        return false;
    }
    options_ = options;
    context_.session = session;
    if (!options.enabled())
        return true;
    try {
        if (!registered_) {
            RegisterBuiltinBtNodes(factory_);
            registered_ = true;
        }
        factory_.clearRegisteredBehaviorTrees();
        auto blackboard = BT::Blackboard::create();
        blackboard->set(keys::kContext, &context_);
        blackboard->set("max_recovery_attempts",
                        options.max_recovery_attempts());
        const std::string name = options.tree().empty()
                                     ? "navigate_with_recovery.xml"
                                     : options.tree();
        const std::string path =
            std::filesystem::path(name).has_parent_path()
                ? common::ResolveConfigPath(name)
                : common::ResolveConfigPath("behavior/trees/" + name);
        tree_ = std::make_unique<BT::Tree>(
            factory_.createTreeFromFile(path, blackboard));
    } catch (const std::exception& error) {
        error_ = error.what();
        tree_.reset();
        return false;
    }
    return true;
}
common::Status BtCppEngine::Tick(std::int64_t now_ns) {
    if (!options_.enabled())
        return {};
    if (!tree_)
        return common::Status::Unavailable("behavior tree is not loaded");
    if (now_ns < last_tick_ns_)
        return common::Status::InvalidArgument("non-monotonic behavior clock");
    last_tick_ns_ = context_.now_ns = now_ns;
    try {
        const auto status = tree_->tickExactlyOnce();
        if (status == BT::NodeStatus::FAILURE || context_.assist_requested)
            return common::Status::Unavailable("behavior recovery exhausted");
    } catch (const std::exception& error) {
        return common::Status::Unavailable(error.what());
    }
    return {};
}
void BtCppEngine::Reset() {
    if (tree_)
        tree_->haltTree();
    context_.assist_requested = false;
    last_tick_ns_ = 0;
}
}  // namespace navigation::behavior
