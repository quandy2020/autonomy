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

/**
 * @file detector.cpp
 * @brief Letterbox infer pipeline and Detector<PromptKind> factories.
 */

#include "autonomy/perception/hestia/detector.hpp"

#include "autonomy/common/network/backend/engine.hpp"
#include "autonomy/common/network/common/options.hpp"
#include "autonomy/perception/hestia/options.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autonomy {
namespace perception {
namespace hestia {

constexpr char kInputName[] = "images";
constexpr char kOutputName[] = "output0";
constexpr float kLetterboxPadding = 114.0F;

void SetPipelineError(std::string* error, const std::string& message) {
    if (error != nullptr) {
        *error = "Hestia detector: " + message;
    }
}

const common::network::ModelTensorInfo* FindTensorInfo(
    const std::vector<common::network::ModelTensorInfo>& infos,
    const char* name) {
    for (const auto& info : infos) {
        if (info.name == name) {
            return &info;
        }
    }
    return nullptr;
}

bool ValidateTensor(const common::network::ModelTensorInfo& info,
                    const char* label, const std::vector<int64_t>& expected,
                    std::string* error) {
    if (info.element_type != common::network::ElementType::kFloat32) {
        SetPipelineError(error, std::string(label) + " must use float32 elements.");
        return false;
    }
    if (info.shape.Rank() != expected.size() || info.shape.Dims() != expected) {
        SetPipelineError(error,
                 std::string(label) + " has an unexpected static shape.");
        return false;
    }
    return true;
}

bool ValidateModelContract(const common::network::Engine& engine, uint32_t width,
                           uint32_t height, uint32_t max_detections,
                           std::string* error) {
    const auto inputs = engine.GetInputInfos();
    const auto outputs = engine.GetOutputInfos();
    const auto* images = FindTensorInfo(inputs, kInputName);
    const auto* output0 = FindTensorInfo(outputs, kOutputName);
    if (inputs.size() != 1 || images == nullptr) {
        SetPipelineError(error, "model input must be exactly 'images'.");
        return false;
    }
    if (outputs.size() != 1 || output0 == nullptr) {
        SetPipelineError(error, "model output must be exactly 'output0'.");
        return false;
    }
    return ValidateTensor(*images, "model input 'images'",
                          {1, 3, static_cast<int64_t>(height),
                           static_cast<int64_t>(width)},
                          error) &&
           ValidateTensor(*output0, "model output 'output0'",
                          {1, static_cast<int64_t>(max_detections), 6}, error);
}

namespace {

class EngineRunner final : public Runner
{
public:
    explicit EngineRunner(
        std::unique_ptr<common::network::Engine> engine)
        : engine_(std::move(engine)) {}

    bool Run(const common::network::TensorMap& inputs,
             common::network::TensorMap* outputs, std::string* error) override {
        if (error != nullptr) {
            error->clear();
        }
        if (outputs == nullptr) {
            if (error != nullptr) {
                *error = "model output map is null.";
            }
            return false;
        }
        outputs->clear();
        if (!engine_->Run(inputs, outputs)) {
            if (error != nullptr) {
                *error = engine_->GetLastError();
            }
            outputs->clear();
            return false;
        }
        return true;
    }

private:
    std::unique_ptr<common::network::Engine> engine_;
};

}  // namespace


bool ValidateImage(const automsgs::msgs::sensor_msgs::Image& image,
                   std::string* error) {
    if (image.encoding() != "rgb8" && image.encoding() != "bgr8") {
        SetPipelineError(error, "image encoding must be 'rgb8' or 'bgr8'.");
        return false;
    }
    if (image.width() == 0 || image.height() == 0) {
        SetPipelineError(error, "image dimensions must be positive.");
        return false;
    }
    if (image.is_bigendian()) {
        SetPipelineError(error, "big-endian images are unsupported.");
        return false;
    }
    if (image.width() >
            static_cast<uint32_t>(std::numeric_limits<int>::max()) ||
        image.height() >
            static_cast<uint32_t>(std::numeric_limits<int>::max())) {
        SetPipelineError(error, "image dimensions exceed OpenCV limits.");
        return false;
    }
    const size_t minimum_step = static_cast<size_t>(image.width()) * 3;
    if (image.step() < minimum_step) {
        SetPipelineError(error, "image step is too small.");
        return false;
    }
    const size_t required_data =
        static_cast<size_t>(image.height()) * image.step();
    if (image.data().size() < required_data) {
        SetPipelineError(error, "image data is too small.");
        return false;
    }
    return true;
}

bool PrepareInput(const automsgs::msgs::sensor_msgs::Image& image,
                  int target_width, int target_height,
                  common::network::TensorMap* tensors, float* scale,
                  int* padding_left, int* padding_top, std::string* error) {
    if (tensors == nullptr || scale == nullptr || padding_left == nullptr ||
        padding_top == nullptr) {
        SetPipelineError(error, "detector preprocessing output is null.");
        return false;
    }
    tensors->clear();
    if (!ValidateImage(image, error)) {
        return false;
    }
    if (target_width <= 0 || target_height <= 0) {
        SetPipelineError(error, "detector dimensions must be positive.");
        return false;
    }

    cv::Mat source(static_cast<int>(image.height()),
                   static_cast<int>(image.width()), CV_8UC3);
    for (uint32_t row = 0; row < image.height(); ++row) {
        std::memcpy(
            source.ptr(static_cast<int>(row)),
            image.data().data() + static_cast<size_t>(row) * image.step(),
            static_cast<size_t>(image.width()) * 3);
    }
    cv::Mat source_rgb;
    if (image.encoding() == "bgr8") {
        cv::cvtColor(source, source_rgb, cv::COLOR_BGR2RGB);
    } else {
        source_rgb = source;
    }

    const float width_scale =
        static_cast<float>(target_width) / static_cast<float>(image.width());
    const float height_scale =
        static_cast<float>(target_height) / static_cast<float>(image.height());
    *scale = std::min(width_scale, height_scale);
    const int resized_width = std::min(
        target_width,
        std::max(1, static_cast<int>(std::round(image.width() * *scale))));
    const int resized_height = std::min(
        target_height,
        std::max(1, static_cast<int>(std::round(image.height() * *scale))));
    const int horizontal_padding = target_width - resized_width;
    const int vertical_padding = target_height - resized_height;
    *padding_left = horizontal_padding / 2;
    *padding_top = vertical_padding / 2;

    cv::Mat resized;
    cv::resize(source_rgb, resized, cv::Size(resized_width, resized_height),
               0.0, 0.0, cv::INTER_LINEAR);
    cv::Mat letterboxed(
        target_height, target_width, CV_8UC3,
        cv::Scalar(kLetterboxPadding, kLetterboxPadding, kLetterboxPadding));
    resized.copyTo(letterboxed(
        cv::Rect(*padding_left, *padding_top, resized_width, resized_height)));

    const size_t plane_size =
        static_cast<size_t>(target_width) * static_cast<size_t>(target_height);
    std::vector<float> data(3 * plane_size);
    for (int row = 0; row < target_height; ++row) {
        for (int col = 0; col < target_width; ++col) {
            const size_t index = static_cast<size_t>(row) * target_width + col;
            const cv::Vec3b& pixel = letterboxed.at<cv::Vec3b>(row, col);
            data[index] = static_cast<float>(pixel[0]) / 255.0F;
            data[plane_size + index] = static_cast<float>(pixel[1]) / 255.0F;
            data[2 * plane_size + index] =
                static_cast<float>(pixel[2]) / 255.0F;
        }
    }
    tensors->emplace(kInputName,
                     common::network::Tensor::FromFloat32(std::move(data)));
    return true;
}

bool OutputFloat32(const common::network::TensorMap& outputs,
                   size_t expected_count, const float** data,
                   std::string* error) {
    const auto output = outputs.find(kOutputName);
    if (output == outputs.end()) {
        SetPipelineError(error, "model output 'output0' is missing.");
        return false;
    }
    std::string detail;
    size_t count = 0;
    if (!output->second.TryViewFloat32(data, &count, &detail)) {
        SetPipelineError(error, "model output 'output0' must be float32: " + detail);
        return false;
    }
    if (count != expected_count) {
        SetPipelineError(error,
                 "model output 'output0' has an unexpected tensor size.");
        return false;
    }
    return true;
}

std::vector<std::string> CopyClassNames(
    const google::protobuf::RepeatedPtrField<std::string>& class_names) {
    return std::vector<std::string>(class_names.begin(), class_names.end());
}

bool InferDetections(const proto::HestiaOptions& options,
               Runner* runner,
               const std::vector<std::string>& class_names, uint32_t width,
               uint32_t height,
               const automsgs::msgs::sensor_msgs::Image& image,
               automsgs::msgs::vision_msgs::Detection2DArray* detections,
               std::string* error) {
    if (error != nullptr) {
        error->clear();
    }
    if (detections != nullptr) {
        detections->Clear();
    }
    if (detections == nullptr || runner == nullptr) {
        SetPipelineError(error, "detection output or runner is null.");
        return false;
    }
    if (width > static_cast<uint32_t>(std::numeric_limits<int>::max()) ||
        height > static_cast<uint32_t>(std::numeric_limits<int>::max())) {
        SetPipelineError(error, "detector dimensions exceed OpenCV limits.");
        return false;
    }
    if (class_names.empty()) {
        SetPipelineError(error, "class name table is empty.");
        return false;
    }

    common::network::TensorMap inputs;
    float scale = 0.0F;
    int padding_left = 0;
    int padding_top = 0;
    if (!PrepareInput(image, static_cast<int>(width), static_cast<int>(height),
                      &inputs, &scale, &padding_left, &padding_top, error)) {
        return false;
    }

    common::network::TensorMap outputs;
    std::string detail;
    if (!runner->Run(inputs, &outputs, &detail)) {
        SetPipelineError(error, detail);
        return false;
    }

    const size_t expected_count =
        static_cast<size_t>(options.max_detections()) * 6;
    const float* values = nullptr;
    if (!OutputFloat32(outputs, expected_count, &values, error)) {
        return false;
    }

    automsgs::msgs::vision_msgs::Detection2DArray parsed;
    *parsed.mutable_header() = image.header();
    const float image_width = static_cast<float>(image.width());
    const float image_height = static_cast<float>(image.height());
    for (uint32_t index = 0; index < options.max_detections(); ++index) {
        const size_t offset = static_cast<size_t>(index) * 6;
        const float x1 = values[offset];
        const float y1 = values[offset + 1];
        const float x2 = values[offset + 2];
        const float y2 = values[offset + 3];
        const float confidence = values[offset + 4];
        const float class_index = values[offset + 5];
        if (!std::isfinite(confidence) || !std::isfinite(class_index) ||
            confidence < options.confidence_threshold()) {
            continue;
        }
        const int label_index = static_cast<int>(std::lround(class_index));
        if (label_index < 0 ||
            label_index >= static_cast<int>(class_names.size())) {
            continue;
        }
        if (!std::isfinite(x1) || !std::isfinite(y1) || !std::isfinite(x2) ||
            !std::isfinite(y2)) {
            continue;
        }

        const float left =
            std::clamp((x1 - padding_left) / scale, 0.0F, image_width);
        const float top =
            std::clamp((y1 - padding_top) / scale, 0.0F, image_height);
        const float right =
            std::clamp((x2 - padding_left) / scale, 0.0F, image_width);
        const float bottom =
            std::clamp((y2 - padding_top) / scale, 0.0F, image_height);
        if (right <= left || bottom <= top) {
            continue;
        }

        auto* detection = parsed.add_detections();
        *detection->mutable_header() = image.header();
        auto* hypothesis = detection->add_results()->mutable_hypothesis();
        hypothesis->set_class_id(class_names[static_cast<size_t>(label_index)]);
        hypothesis->set_score(confidence);
        auto* bbox = detection->mutable_bbox();
        bbox->mutable_center()->mutable_position()->set_x((left + right) /
                                                          2.0F);
        bbox->mutable_center()->mutable_position()->set_y((top + bottom) /
                                                          2.0F);
        bbox->set_size_x(right - left);
        bbox->set_size_y(bottom - top);
    }

    if (options.nms_iou_threshold() > 0.0F && parsed.detections_size() > 1) {
        using Detection2D = automsgs::msgs::vision_msgs::Detection2D;
        std::vector<Detection2D> boxes(parsed.detections().begin(),
                                       parsed.detections().end());
        std::sort(boxes.begin(), boxes.end(),
                  [](const Detection2D& lhs, const Detection2D& rhs) {
                      const double ls =
                          lhs.results().empty() ? 0.0
                                                : lhs.results(0).hypothesis().score();
                      const double rs =
                          rhs.results().empty() ? 0.0
                                                : rhs.results(0).hypothesis().score();
                      return ls > rs;
                  });
        auto BoxIoU = [](const Detection2D& lhs, const Detection2D& rhs) {
            const auto& a = lhs.bbox();
            const auto& b = rhs.bbox();
            const double a_left = a.center().position().x() - a.size_x() * 0.5;
            const double a_top = a.center().position().y() - a.size_y() * 0.5;
            const double a_right = a.center().position().x() + a.size_x() * 0.5;
            const double a_bottom = a.center().position().y() + a.size_y() * 0.5;
            const double b_left = b.center().position().x() - b.size_x() * 0.5;
            const double b_top = b.center().position().y() - b.size_y() * 0.5;
            const double b_right = b.center().position().x() + b.size_x() * 0.5;
            const double b_bottom = b.center().position().y() + b.size_y() * 0.5;
            const double iw = std::max(
                0.0, std::min(a_right, b_right) - std::max(a_left, b_left));
            const double ih = std::max(
                0.0, std::min(a_bottom, b_bottom) - std::max(a_top, b_top));
            const double intersection = iw * ih;
            const double union_area =
                a.size_x() * a.size_y() + b.size_x() * b.size_y() - intersection;
            return union_area > 0.0 ? intersection / union_area : 0.0;
        };
        std::vector<bool> suppressed(boxes.size(), false);
        const double thr = options.nms_iou_threshold();
        parsed.clear_detections();
        for (size_t i = 0; i < boxes.size(); ++i) {
            if (suppressed[i]) {
                continue;
            }
            *parsed.add_detections() = boxes[i];
            for (size_t j = i + 1; j < boxes.size(); ++j) {
                if (!suppressed[j] && BoxIoU(boxes[i], boxes[j]) >= thr) {
                    suppressed[j] = true;
                }
            }
        }
    }

    *detections = std::move(parsed);
    return true;
}

std::unique_ptr<Runner> CreateEngineRunner(
    const std::string& backend, const std::string& model_path, uint32_t width,
    uint32_t height, uint32_t max_detections, std::string* error) {
    common::network::InferenceOptions inference_options;
    inference_options.backend_id = backend;
    inference_options.model_path = model_path;
    std::string engine_error;
    auto engine =
        common::network::Engine::CreateEngine(inference_options, &engine_error);
    if (engine == nullptr) {
        SetPipelineError(error, engine_error);
        return nullptr;
    }
    if (!ValidateModelContract(*engine, width, height, max_detections,
                               error)) {
        return nullptr;
    }
    return std::make_unique<EngineRunner>(std::move(engine));
}


template <PromptKind K>
std::unique_ptr<Detector<K>> Detector<K>::Create(
    const proto::HestiaOptions& options, std::string* error) {
    proto::HestiaOptions normalized = Detector<K>::Normalize(options);
    if (error != nullptr) {
        error->clear();
    }
    if (!ValidateHestiaOptions(normalized, error)) {
        return nullptr;
    }
    auto runner = CreateEngineRunner(
        BackendId(normalized.backend()), Detector<K>::ModelPath(normalized),
        Detector<K>::Width(normalized), Detector<K>::Height(normalized),
        normalized.max_detections(), error);
    if (runner == nullptr) {
        return nullptr;
    }
    return Create(normalized, std::move(runner), error);
}

template <PromptKind K>
std::unique_ptr<Detector<K>> Detector<K>::Create(
    const proto::HestiaOptions& options,
    std::unique_ptr<Runner> runner, std::string* error) {
    proto::HestiaOptions normalized = Detector<K>::Normalize(options);
    if (error != nullptr) {
        error->clear();
    }
    if (!ValidateHestiaOptions(normalized, error)) {
        return nullptr;
    }
    if (runner == nullptr) {
        SetPipelineError(error, "inference runner is null.");
        return nullptr;
    }
    return std::unique_ptr<Detector<K>>(new Detector<K>(
        normalized, std::move(runner),
        CopyClassNames(Detector<K>::ClassNames(normalized)),
        Detector<K>::Width(normalized), Detector<K>::Height(normalized)));
}

template class Detector<PromptKind::OpenVocabulary>;
template class Detector<PromptKind::ClosedSet>;

}  // namespace hestia
}  // namespace perception
}  // namespace autonomy
