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
#include "autonomy/perception/tasks/classification/classification_task.hpp"

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
namespace classify {

bool Decode(const common::network::Engine& engine,
            const proto::BaseOptions& options,
            const automsgs::msgs::sensor_msgs::Image& rgb, Outputs* outputs,
            std::string* error) {
    if (!outputs) {
        SetTaskError(error, "Missing task output.");
        return false;
    }
    common::network::RunResult result;
    if (!RunImageInference(engine, options, rgb, common::network::ResizePolicy::kCenterCrop,
                           &result, error))
        return false;
    const float* data = nullptr;
    size_t count = 0;
    std::vector<int64_t> shape;
    const auto name = OutputName(options, proto::TASK_CLASSIFY, "output0");
    if (!ReadPrimaryOutput(result, engine.GetOutputInfos(), name, &data, &count, &shape, error))
        return false;
    if (shape.size() != 2 || shape[0] != 1 ||
        count != static_cast<size_t>(options.class_names_size())) {
        SetTaskError(error, "Classification requires output0 [1,C] matching class_names.");
        return false;
    }
    std::vector<size_t> indices;
    for (size_t i = 0; i < count; ++i) {
        if (!std::isfinite(data[i]) || data[i] < 0 || data[i] > 1) {
            SetTaskError(error, "Classification output must contain probabilities.");
            return false;
        }
        if (data[i] >= options.confidence_threshold())
            indices.push_back(i);
    }
    std::stable_sort(indices.begin(), indices.end(),
                     [&](size_t a, size_t b) { return data[a] > data[b]; });
    auto& classification = outputs->classification;
    classification.Clear();
    classification.mutable_header()->CopyFrom(rgb.header());
    for (size_t i = 0; i < std::min<size_t>(indices.size(), options.max_detections()); ++i) {
        const size_t index = indices[i];
        auto* hypothesis = classification.add_results();
        hypothesis->set_class_id(options.class_names(static_cast<int>(index)));
        hypothesis->set_score(data[index]);
    }
    return true;
}

}  // namespace classify
}  // namespace base
}  // namespace perception
}  // namespace autonomy
