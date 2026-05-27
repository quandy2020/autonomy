/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#pragma once

#include <memory>
#include <string>

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/tasks/behavior_tree/behavior_tree_context.hpp"
#include "autonomy/tasks/behavior_tree/behavior_tree_engine.hpp"
#include "autonomy/tasks/proto/task_options.pb.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class NavigatorInterface
{
public:
    virtual ~NavigatorInterface() = default;

    virtual std::string GetName() const = 0;

    virtual bool Configure(std::shared_ptr<BehaviorTreeEngine> engine,
                           std::shared_ptr<BehaviorTreeContext> ctx,
                           const std::string& default_behavior_tree_file) = 0;

    virtual bool Start(const std::string& bt_xml_file) = 0;
    virtual bool Cancel() = 0;
    virtual bool Pause() = 0;
    virtual bool Resume() = 0;
    virtual bool IsRunning() const = 0;
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
