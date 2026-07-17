/*
 * Copyright 2026 The Openbot Authors
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

#include <functional>
#include <memory>

#include "autolink/node/node.hpp"
#include "autonomy/common/macros.hpp"
#include "autonomy/task/proto/teleop.pb.h"

namespace autonomy {
namespace task {

class TaskServer;

/** Subscribes to {@link kTeleopGoalChannel} and forwards goals to TaskServer. */
class TeleopGoalIngress
{
public:
    using SubmitFn = std::function<bool(
        const ::autonomy::task::proto::TeleopGoal&)>;

    AUTONOMY_SMART_PTR_DEFINITIONS(TeleopGoalIngress)

    TeleopGoalIngress() = default;
    ~TeleopGoalIngress();

    bool Start(const std::shared_ptr<autolink::Node>& node, SubmitFn submit,
               std::function<void(const ::autonomy::task::proto::TeleopGoal&)>
                   on_rejected = nullptr);
    void Stop();

private:
    void OnGoal(const std::shared_ptr<::autonomy::task::proto::TeleopGoal>& goal);

    SubmitFn submit_;
    std::function<void(const ::autonomy::task::proto::TeleopGoal&)> on_rejected_;
    std::shared_ptr<autolink::Node> node_;
    std::shared_ptr<autolink::Reader<::autonomy::task::proto::TeleopGoal>>
        reader_;
};

}  // namespace task
}  // namespace autonomy
