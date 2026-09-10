/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
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

#include "autonomy/perception/base/tasks/depth/depth.hpp"

#include "autonomy/perception/base/tasks/depth/moge.hpp"
#include "autonomy/perception/base/tasks/task.hpp"
#include "autonomy/perception/base/options.hpp"

namespace autonomy {
namespace perception {
namespace base {
namespace depth {

bool Decode(const common::network::Engine& engine,
            const proto::BaseOptions& options,
            const automsgs::msgs::sensor_msgs::Image& rgb, Outputs* outputs,
            std::string* error) {
    if (outputs == nullptr) {
        SetTaskError(error, "outputs must not be null.");
        return false;
    }
    const auto* task = FindTask(options, proto::TASK_DEPTH);
    if (task == nullptr) {
        SetTaskError(error, "TASK_DEPTH options are missing.");
        return false;
    }
    switch (task->depth_backend()) {
        case proto::DEPTH_BACKEND_MOGE:
            return moge::Decode(engine, options, rgb, outputs, error);
        default:
            SetTaskError(error,
                         "depth_backend must be DEPTH_BACKEND_MOGE for "
                         "monocular depth.");
            return false;
    }
}

}  // namespace depth
}  // namespace base
}  // namespace perception
}  // namespace autonomy
