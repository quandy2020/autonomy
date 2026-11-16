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
#include "autonomy/perception/inference/image_input.hpp"
#include "autonomy/perception/tasks/semantic_seg/semantic_seg_task.hpp"

#include "autonomy/common/network/network.hpp"
#include "autonomy/perception/tasks/task_base.hpp"

#include <opencv2/imgproc.hpp>

#include <cmath>
#include <limits>
#include <vector>

namespace autonomy::perception::base::semantic {

bool Decode(const common::network::Engine& engine,
            const proto::BaseOptions& options,
            const automsgs::msgs::sensor_msgs::Image& rgb, Outputs* outputs,
            std::string* error) {
    if (!outputs || rgb.width() == 0 || rgb.height() == 0 ||
        rgb.width() > std::numeric_limits<int>::max() ||
        rgb.height() > std::numeric_limits<int>::max() ||
        rgb.step() < 3ULL * rgb.width() ||
        rgb.data().size() < static_cast<uint64_t>(rgb.step()) * rgb.height() ||
        (rgb.encoding() != "rgb8" && rgb.encoding() != "bgr8")) {
        SetTaskError(error, "Invalid semantic input image.");
        return false;
    }

    cv::Mat image(rgb.height(), rgb.width(), CV_8UC3,
                  const_cast<char*>(rgb.data().data()), rgb.step());
    auto prep = common::network::Stretch(options.input_height(),
                                         options.input_width());
    prep.normalize = common::network::NormalizePolicy::kImageNet;
    // The shared ImageNet normalization expects BGR before its channel swap.
    cv::Mat bgr;
    if (rgb.encoding() == "rgb8") {
        cv::cvtColor(image, bgr, cv::COLOR_RGB2BGR);
    } else {
        bgr = image;
    }
    common::network::RunResult result;
    if (!common::network::RunPipeline(
            const_cast<common::network::Engine*>(&engine), bgr, prep,
            &result, error)) {
        return false;
    }

    const auto infos = engine.GetOutputInfos();
    const auto name = OutputName(options, proto::TASK_SEMANTIC, "logits");
    const float* data = nullptr;
    size_t count = 0;
    std::vector<int64_t> shape;
    if (!ReadPrimaryOutput(result, infos, name, &data, &count, &shape, error))
        return false;
    if (shape.size() != 4 || shape[0] != 1 ||
        shape[1] < 2 || shape[1] > 255 ||
        shape[2] <= 0 || shape[2] > std::numeric_limits<int>::max() ||
        shape[3] <= 0 || shape[3] > std::numeric_limits<int>::max()) {
        SetTaskError(error, "Semantic model requires float logits [1,C,H,W], C in [2,255].");
        return false;
    }

    cv::Mat labels(static_cast<int>(shape[2]), static_cast<int>(shape[3]),
                   CV_8UC1);
    const size_t pixels = labels.total();
    for (size_t pixel = 0; pixel < pixels; ++pixel) {
        float best = -std::numeric_limits<float>::infinity();
        uint8_t label = 255;
        for (int64_t channel = 0; channel < shape[1]; ++channel) {
            const float value = data[static_cast<size_t>(channel) * pixels + pixel];
            if (!std::isfinite(value)) {
                label = 255;
                break;
            }
            if (value > best) {
                best = value;
                label = static_cast<uint8_t>(channel);
            }
        }
        labels.data[pixel] = label;
    }
    cv::Mat full_size;
    cv::resize(labels, full_size, image.size(), 0, 0, cv::INTER_NEAREST);
    auto& message = outputs->semantic;
    message.Clear();
    message.set_width(rgb.width());
    message.set_height(rgb.height());
    message.set_encoding("mono8");
    message.set_step(rgb.width());
    message.set_data(reinterpret_cast<const char*>(full_size.data),
                     full_size.total());
    return true;
}

}  // namespace autonomy::perception::base::semantic
