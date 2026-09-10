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

#include "autonomy/perception/base/tasks/detect/detect.hpp"

#include "autonomy/perception/base/tasks/task.hpp"

namespace autonomy {
namespace perception {
namespace base {
namespace detect {

bool Decode(const common::network::Engine& /*engine*/,
            const proto::BaseOptions& /*options*/,
            const automsgs::msgs::sensor_msgs::Image& /*rgb*/, Outputs* outputs,
            std::string* error) {
    if (outputs == nullptr) {
        SetTaskError(error, "outputs must not be null.");
        return false;
    }
    SetTaskError(error,
                 "YOLO26 detect decoder is not bound. Export end2end "
                 "output0 [1,max_det,6] as x1,y1,x2,y2,score,class_index.");
    return false;
}

}  // namespace detect
}  // namespace base
}  // namespace perception
}  // namespace autonomy
