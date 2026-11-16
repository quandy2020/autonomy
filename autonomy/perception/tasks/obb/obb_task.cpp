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

#include "autonomy/perception/engine/binding.hpp"
#include "autonomy/perception/tasks/obb/obb_task.hpp"

#include "autonomy/perception/tasks/task_base.hpp"
#include "autonomy/perception/inference/image_input.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <opencv2/core.hpp>

namespace autonomy {
namespace perception {
namespace base {
namespace obb {

bool Decode(const common::network::Engine& engine,
            const proto::BaseOptions& options,
            const automsgs::msgs::sensor_msgs::Image& rgb, Outputs* outputs,
            std::string* error) {
    if (!outputs) {
        SetTaskError(error, "Missing task output.");
        return false;
    }
    common::network::RunResult result;
    if (!RunImageInference(engine, options, rgb, common::network::ResizePolicy::kLetterbox,
                           &result, error))
        return false;
    const float* data = nullptr;
    size_t count = 0;
    std::vector<int64_t> shape;
    const auto name = OutputName(options, proto::TASK_OBB, "output0");
    if (!ReadPrimaryOutput(result, engine.GetOutputInfos(), name, &data, &count, &shape, error))
        return false;
    const double scale = result.meta.scale_gain;
    if (shape.size() != 3 || shape[0] != 1 || shape[2] != 7 ||
        !std::isfinite(scale) || scale <= 0) {
        SetTaskError(error, "OBB requires output0 [1,N,7].");
        return false;
    }
    auto& detections = outputs->obb;
    detections.Clear();
    detections.mutable_header()->CopyFrom(rgb.header());
    for (size_t i = 0; i < count; i += 7) {
        const auto* row = data + i;
        if (!std::all_of(row, row + 7, [](float value) { return std::isfinite(value); }) ||
            row[2] <= 0 || row[3] <= 0 ||
            row[5] < options.confidence_threshold() || row[5] > 1 ||
            row[6] < 0 || static_cast<double>(row[6]) >= options.class_names_size() ||
            std::floor(row[6]) != row[6]) {
            continue;
        }
        const double x = (row[0] - result.meta.padding_left) / scale;
        const double y = (row[1] - result.meta.padding_top) / scale;
        if (x < 0 || x >= rgb.width() || y < 0 || y >= rgb.height())
            continue;
        auto* detection = detections.add_detections();
        detection->mutable_header()->CopyFrom(rgb.header());
        auto* box = detection->mutable_bbox();
        box->mutable_center()->mutable_position()->set_x(x);
        box->mutable_center()->mutable_position()->set_y(y);
        box->mutable_center()->set_theta(row[4]);
        box->set_size_x(row[2] / scale);
        box->set_size_y(row[3] / scale);
        auto* hypothesis = detection->add_results()->mutable_hypothesis();
        hypothesis->set_class_id(options.class_names(static_cast<int>(row[6])));
        hypothesis->set_score(row[5]);
        if (static_cast<uint32_t>(detections.detections_size()) >= options.max_detections())
            break;
    }
    return true;
}

}  // namespace obb
}  // namespace base
}  // namespace perception
}  // namespace autonomy
