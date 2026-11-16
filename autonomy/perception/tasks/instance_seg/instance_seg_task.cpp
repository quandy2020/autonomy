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
#include "autonomy/perception/tasks/instance_seg/instance_seg_task.hpp"

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
namespace segment {

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
    const auto infos = engine.GetOutputInfos();
    const float* data = nullptr;
    size_t count = 0;
    std::vector<int64_t> shape;
    const auto name = OutputName(options, proto::TASK_SEGMENT, "output0");
    if (!ReadPrimaryOutput(result, infos, name, &data, &count, &shape, error))
        return false;
    if (shape.size() != 3 || shape[0] != 1 || shape[2] != 6 ||
        (!std::isfinite(result.meta.scale_gain) || result.meta.scale_gain <= 0.0)) {
        SetTaskError(error, "YOLO26 requires output0 shaped [1,N,6].");
        return false;
    }

    const auto* task = FindTask(options, proto::TASK_SEGMENT);
    const bool publish_masks = task && !task->instance_masks_topic().empty();
    const float* masks = nullptr;
    size_t mask_count = 0;
    const int height = result.meta.input_height;
    const int width = result.meta.input_width;
    if (publish_masks) {
        const auto mask_name = task->mask_output_name().empty()
            ? "masks" : task->mask_output_name();
        const auto tensor = result.outputs.find(mask_name);
        const auto metadata = std::find_if(infos.begin(), infos.end(),
            [&](const auto& item) { return item.name == mask_name; });
        std::vector<int64_t> dims;
        if (height <= 0 || width <= 0 ||
            tensor == result.outputs.end() || metadata == infos.end() ||
            !tensor->second.TryViewFloat32(&masks, &mask_count, error) ||
            !masks ||
            !common::network::ResolveShapeForElementCount(
                *metadata, mask_count, &dims, error) ||
            dims != std::vector<int64_t>{1, shape[1], height, width}) {
            SetTaskError(error, "Segment requires masks [1,N,input_height,input_width].");
            return false;
        }
    }

    outputs->instance_masks.Clear();
    outputs->instance_masks.mutable_header()->CopyFrom(rgb.header());
    auto& detections = outputs->masks;
    detections.Clear();
    detections.mutable_header()->CopyFrom(rgb.header());
    for (size_t i = 0; i < count; i += 6) {
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
        if (publish_masks) {
            const size_t plane = static_cast<size_t>(height) * width;
            const auto* values = masks + (i / 6) * plane;
            auto* image = outputs->instance_masks.add_masks();
            image->set_width(rgb.width());
            image->set_height(rgb.height());
            image->set_step(rgb.width());
            image->set_encoding("mono8");
            auto& bytes = *image->mutable_data();
            bytes.resize(static_cast<size_t>(rgb.width()) * rgb.height(), 0);
            for (uint32_t y = 0; y < rgb.height(); ++y) {
                if (y < y1 || y >= y2)
                    continue;
                const auto sy = static_cast<int>(y * result.meta.scale_gain +
                                                result.meta.padding_top);
                if (sy < 0 || sy >= height)
                    continue;
                for (uint32_t x = 0; x < rgb.width(); ++x) {
                    if (x < x1 || x >= x2)
                        continue;
                    const auto sx = static_cast<int>(x * result.meta.scale_gain +
                                                    result.meta.padding_left);
                    if (sx >= 0 && sx < width &&
                        values[static_cast<size_t>(sy) * width + sx] >= 0.5F) {
                        bytes[static_cast<size_t>(y) * rgb.width() + x] =
                            static_cast<char>(255);
                    }
                }
            }
        }
        if (static_cast<uint32_t>(detections.detections_size()) >= options.max_detections())
            break;
    }
    return true;
}

}  // namespace segment
}  // namespace base
}  // namespace perception
}  // namespace autonomy
