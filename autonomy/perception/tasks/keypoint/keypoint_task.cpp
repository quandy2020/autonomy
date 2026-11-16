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
#include "autonomy/perception/tasks/keypoint/keypoint_task.hpp"

#include "autonomy/perception/tasks/task_base.hpp"
#include "autonomy/perception/engine/options.hpp"
#include "autonomy/perception/inference/image_input.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#include <opencv2/imgproc.hpp>

namespace autonomy {
namespace perception {
namespace base {
namespace pose {

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
    const auto* task = FindTask(options, proto::TASK_POSE);
    if (!task || !task->num_keypoints()) {
        SetTaskError(error, "Pose keypoint count is missing.");
        return false;
    }
    const int64_t columns = 6 + 3LL * task->num_keypoints();
    const float* data = nullptr;
    size_t count = 0;
    std::vector<int64_t> shape;
    const auto name = OutputName(options, proto::TASK_POSE, "output0");
    if (!ReadPrimaryOutput(result, engine.GetOutputInfos(), name, &data, &count, &shape, error))
        return false;
    if (shape.size() != 3 || shape[0] != 1 || shape[2] != columns ||
        !(result.meta.scale_gain > 0.0)) {
        SetTaskError(error, "Pose requires output0 shaped [1,N,6+3K].");
        return false;
    }

    auto& detections = outputs->poses;
    detections.Clear();
    outputs->keypoints.Clear();
    outputs->keypoints.mutable_header()->CopyFrom(rgb.header());
    detections.mutable_header()->CopyFrom(rgb.header());
    for (size_t i = 0; i < count; i += static_cast<size_t>(columns)) {
        const auto* row = data + i;
        if (!std::all_of(row, row + 6, [](float value) { return std::isfinite(value); }) ||
            row[4] < options.confidence_threshold() || row[4] > 1 ||
            row[5] < 0 || static_cast<double>(row[5]) >= options.class_names_size() ||
            std::floor(row[5]) != row[5]) {
            continue;
        }
        const auto map = [&](float coord, int pad, uint32_t extent) {
            return std::clamp((coord - pad) / result.meta.scale_gain, 0.0,
                              static_cast<double>(extent));
        };
        const double x1 = map(row[0], result.meta.padding_left, rgb.width());
        const double y1 = map(row[1], result.meta.padding_top, rgb.height());
        const double x2 = map(row[2], result.meta.padding_left, rgb.width());
        const double y2 = map(row[3], result.meta.padding_top, rgb.height());
        if (x2 <= x1 || y2 <= y1)
            continue;
        auto* detection = detections.add_detections();
        detection->mutable_header()->CopyFrom(rgb.header());
        auto* box = detection->mutable_bbox();
        box->mutable_center()->mutable_position()->set_x((x1 + x2) / 2);
        box->mutable_center()->mutable_position()->set_y((y1 + y2) / 2);
        box->set_size_x(x2 - x1);
        box->set_size_y(y2 - y1);
        auto* hypothesis = detection->add_results()->mutable_hypothesis();
        hypothesis->set_class_id(options.class_names(static_cast<int>(row[5])));
        hypothesis->set_score(row[4]);
        auto* person = outputs->keypoints.add_people();
        for (int64_t column = 6; column < columns; column += 3) {
            auto* point = person->add_points();
            const float confidence = row[column + 2];
            if (!(confidence > 0 && confidence <= 1) ||
                !std::isfinite(row[column]) || !std::isfinite(row[column + 1]))
                continue;
            point->set_x(map(row[column], result.meta.padding_left, rgb.width()));
            point->set_y(map(row[column + 1], result.meta.padding_top, rgb.height()));
            point->set_score(confidence);
        }
        if (static_cast<uint32_t>(detections.detections_size()) >= options.max_detections())
            break;
    }
    return true;
}

}  // namespace pose
}  // namespace base
}  // namespace perception
}  // namespace autonomy
