/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "autonomy/control/controller/neupan_controller/distance_encoder.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace neupan_controller {

DistanceEncoder::DistanceEncoder(
    const Mat& G, const Vec& h,
    std::unique_ptr<::autonomy::common::network::Engine> engine,
    std::string input_name, std::string output_name)
    : halfspace_matrix_(G),
      halfspace_offset_(h),
      engine_(std::move(engine)),
      input_name_(std::move(input_name)),
      output_name_(std::move(output_name)) {}

std::unique_ptr<DistanceEncoder> DistanceEncoder::Create(
    const std::string& model_path, const Mat& halfspace_G, const Vec& halfspace_h,
    const std::string& inference_backend,
    const std::string& inference_execution_provider, std::string* error) {
    if (model_path.empty()) {
        if (error) {
            *error = "DistanceEncoder: neural_encoder_model_path is empty";
        }
        return nullptr;
    }
    if (halfspace_h.size() <= 0) {
        if (error) {
            *error = "DistanceEncoder: empty halfspaces";
        }
        return nullptr;
    }

    ::autonomy::common::network::InferenceOptions options;
    options.backend_id = inference_backend.empty() ? "onnx" : inference_backend;
    options.model_path = model_path;
    options.onnx.execution_provider = inference_execution_provider;

    auto engine = ::autonomy::common::network::Engine::CreateEngine(options, error);
    if (!engine) {
        return nullptr;
    }

    const auto input_infos = engine->GetInputInfos();
    const auto output_infos = engine->GetOutputInfos();
    if (input_infos.empty() || output_infos.empty()) {
        if (error) {
            *error = "DistanceEncoder: model has no inputs/outputs";
        }
        return nullptr;
    }

    const auto& output_shape = output_infos.front().shape.Dims();
    if (!output_shape.empty()) {
        const int64_t last_dimension = output_shape.back();
        if (last_dimension > 0 && last_dimension != halfspace_h.size()) {
            if (error) {
                *error = "DistanceEncoder: ONNX output channels (" +
                         std::to_string(last_dimension) +
                         ") != footprint edge count (" +
                         std::to_string(halfspace_h.size()) + ")";
            }
            return nullptr;
        }
    }

    std::string warmup_error;
    engine->Warmup(&warmup_error);
    return std::unique_ptr<DistanceEncoder>(new DistanceEncoder(
        halfspace_G, halfspace_h, std::move(engine), input_infos.front().name,
        output_infos.front().name));
}

bool DistanceEncoder::InferDuals(const Mat2X& points_in_robot, Mat* mu,
                                 Vec* distances, std::string* error) const {
    if (mu == nullptr || distances == nullptr || !engine_) {
        if (error) {
            *error = "DistanceEncoder::InferDuals invalid state";
        }
        return false;
    }
    const int point_count = static_cast<int>(points_in_robot.cols());
    const int edge_count = static_cast<int>(halfspace_offset_.size());
    if (point_count <= 0) {
        mu->resize(edge_count, 0);
        distances->resize(0);
        return true;
    }

    input_buffer_.resize(static_cast<size_t>(point_count) * 2u);
    for (int i = 0; i < point_count; ++i) {
        input_buffer_[static_cast<size_t>(i) * 2u] =
            static_cast<float>(points_in_robot(0, i));
        input_buffer_[static_cast<size_t>(i) * 2u + 1u] =
            static_cast<float>(points_in_robot(1, i));
    }
    inputs_.clear();
    outputs_.clear();
    inputs_[input_name_] = std::move(input_buffer_);
    {
        std::lock_guard<std::mutex> lock(inference_mutex_);
        if (!engine_->Run(inputs_, &outputs_)) {
            if (error) {
                *error = engine_->GetLastError();
                if (error->empty()) {
                    *error = "DistanceEncoder Engine::Run failed";
                }
            }
            input_buffer_ = std::move(inputs_[input_name_]);
            return false;
        }
    }
    input_buffer_ = std::move(inputs_[input_name_]);

    const auto output_it = outputs_.find(output_name_);
    if (output_it == outputs_.end() ||
        static_cast<int>(output_it->second.size()) < point_count * edge_count) {
        if (error) {
            *error = "DistanceEncoder: missing or short output tensor";
        }
        return false;
    }

    mu->resize(edge_count, point_count);
    const float* mu_data = output_it->second.data();
    for (int i = 0; i < point_count; ++i) {
        const float* row =
            mu_data + static_cast<size_t>(i) * static_cast<size_t>(edge_count);
        for (int edge = 0; edge < edge_count; ++edge) {
            (*mu)(edge, i) = static_cast<double>(row[edge]);
        }
    }
    Mat residual = halfspace_matrix_ * points_in_robot;
    residual.colwise() -= halfspace_offset_;
    distances->resize(point_count);
    distances->noalias() =
        (mu->cwiseProduct(residual)).colwise().sum().transpose();
    return true;
}

}  // namespace neupan_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
