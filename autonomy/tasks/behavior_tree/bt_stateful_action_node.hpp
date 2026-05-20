/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <memory>
#include <string>

#include "autonomy/common/logging.hpp"
#include "autonomy/tasks/behavior_tree/behavior_tree_utils.hpp"
#include "autonomy/tasks/common/task_context.hpp"
#include "behaviortree_cpp/action_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

/**
 * @brief In-process BT action node: no ActionClient / topic / service stub.
 */
class BtStatefulActionNode : public BT::ActionNodeBase
{
public:
    BtStatefulActionNode(const std::string& xml_tag_name,
                         const BT::NodeConfiguration& conf)
        : BT::ActionNodeBase(xml_tag_name, conf) {}

    BtStatefulActionNode() = delete;
    virtual ~BtStatefulActionNode() = default;

    BT::NodeStatus tick() override {
        if (!BT::isStatusActive(status())) {
            setStatus(BT::NodeStatus::RUNNING);
            started_ = false;
        }

        if (!started_) {
            started_ = true;
            const BT::NodeStatus start_status = onStart();
            if (start_status != BT::NodeStatus::RUNNING) {
                return start_status;
            }
        }

        if (isCancelRequested()) {
            onHalted();
            return BT::NodeStatus::FAILURE;
        }

        return onRunning();
    }

    void halt() override {
        onHalted();
        started_ = false;
        resetStatus();
    }

protected:
    virtual BT::NodeStatus onStart() {
        return BT::NodeStatus::RUNNING;
    }

    virtual BT::NodeStatus onRunning() = 0;

    virtual void onHalted() {}

    std::shared_ptr<common::TaskContext> taskContext() const {
        return config().blackboard->get<std::shared_ptr<common::TaskContext>>(
            "task_context");
    }

    bool isCancelRequested() const {
        auto ctx = taskContext();
        return ctx && ctx->IsCancelRequested();
    }

    void incrementRecoveryCount() {
        if (!config().blackboard) {
            return;
        }
        int recovery_count = 0;
        config().blackboard->get("number_recoveries", recovery_count);  // NOLINT
        config().blackboard->set("number_recoveries", recovery_count + 1);  // NOLINT
    }

private:
    bool started_{false};
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
