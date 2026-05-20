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

#include "autonomy/control/controller_server.hpp"
#include "autonomy/map/map_server.hpp"
#include "autonomy/planning/planner_server.hpp"
#include "autonomy/planning/smoother_server.hpp"
#include "autonomy/system/proto/autonomy_options.pb.h"
#include "autonomy/tasks/common/task_context.hpp"
#include "autonomy/transform/buffer.hpp"

namespace autonomy {
namespace system {

class AutonomyNode
{
public:
    using TfBuffer = autonomy::transform::Buffer;

    AUTONOMY_SMART_PTR_DEFINITIONS(AutonomyNode)

    AutonomyNode() = default;

    explicit AutonomyNode(const proto::AutonomyOptions& options);

    ~AutonomyNode();

    void Start();

    void Shutdown();

    map::MapServer* map_server() {
        return map_server_.get();
    }

    planning::PlannerServer* planner_server() {
        return planner_server_.get();
    }

    planning::SmootherServer* smoother_server() {
        return smoother_server_.get();
    }

    tasks::common::TaskContext* task_context() {
        return task_context_.get();
    }

    control::ControllerServer* controller_server() {
        return controller_server_.get();
    }

private:
    proto::AutonomyOptions options_;

    TfBuffer* tf_buffer_{nullptr};

    map::MapServer::SharedPtr map_server_{nullptr};

    control::ControllerServer::SharedPtr controller_server_{nullptr};

    planning::PlannerServer::SharedPtr planner_server_{nullptr};

    planning::SmootherServer::SharedPtr smoother_server_{nullptr};

    std::shared_ptr<tasks::common::TaskContext> task_context_{nullptr};
};

AutonomyNode::UniquePtr CreateAutonomy(const proto::AutonomyOptions& options);

}  // namespace system
}  // namespace autonomy
