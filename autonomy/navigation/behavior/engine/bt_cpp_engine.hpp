/* Copyright 2026 The Navigation Authors (navigation) */
#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <memory>
#include "navigation/behavior/engine/bt_engine.hpp"
#include "navigation/behavior/nodes/node_context.hpp"

namespace navigation::behavior {

/** @brief Runtime XML engine with the same registration path used by tests. */
class BtCppEngine final : public BtEngine
{
public:
    bool Configure(const config::BehaviorOptions& options,
                   planning::NavigationSession* session) override;
    common::Status Tick(std::int64_t now_ns) override;
    void Reset() override;
    std::string_view Name() const override {
        return "bt_cpp";
    }
    NodeContext& Context() {
        return context_;
    }
    const std::string& Error() const {
        return error_;
    }

private:
    BT::BehaviorTreeFactory factory_;
    std::unique_ptr<BT::Tree> tree_;
    NodeContext context_;
    config::BehaviorOptions options_;
    std::string error_;
    std::int64_t last_tick_ns_{0};
    bool registered_{false};
};
}  // namespace navigation::behavior
