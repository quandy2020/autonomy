/*
 * Copyright 2026 The Openbot Authors (duyongquan)
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

#include "autonomy/control/controller/neupan_controller/distance_encoder.hpp"

#include <limits>

namespace autonomy {
namespace control {
namespace controller {
namespace neupan_controller {

DistanceEncoder::DistanceEncoder(
    const RobotFootprint& footprint,
    std::unique_ptr<::autonomy::common::network::Engine> engine, std::string input_name,
    std::string output_name)
    : halfspace_matrix_(footprint.halfspace_matrix),
      halfspace_matrix_AT_(footprint.halfspace_matrix.transpose()),
      halfspace_offset_(footprint.halfspace_offset),
      engine_(std::move(engine)),
      input_name_(std::move(input_name)),
      output_name_(std::move(output_name)) {}

std::unique_ptr<DistanceEncoder> DistanceEncoder::Create(
    const std::string& model_path, const RobotFootprint& footprint,
    const std::string& inference_backend,
    const std::string& inference_execution_provider, std::string* error) {
    if (model_path.empty()) {
        if (error) {
            *error = "DistanceEncoder: neural_encoder_model_path is empty";
        }
        return nullptr;
    }
    if (footprint.GetEdgeCount() <= 0) {
        if (error) {
            *error = "DistanceEncoder: invalid robot footprint edge count";
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
        if (last_dimension > 0 && last_dimension != footprint.GetEdgeCount()) {
            if (error) {
                *error = "DistanceEncoder: ONNX output channels (" +
                         std::to_string(last_dimension) +
                         ") != footprint edge count (" +
                         std::to_string(footprint.GetEdgeCount()) +
                         "). Retrain/export for this footprint.";
            }
            return nullptr;
        }
    }

    std::string warmup_error;
    engine->Warmup(&warmup_error);

    return std::unique_ptr<DistanceEncoder>(new DistanceEncoder(
        footprint, std::move(engine), input_infos.front().name,
        output_infos.front().name));
}

bool DistanceEncoder::ComputeDistanceFeatures(
    const Eigen::Matrix2Xd& points_in_robot,
    const Eigen::Matrix2d& world_from_robot_rotation,
    DistanceFeatures* features, std::string* error) const {
    if (features == nullptr || !engine_) {
        if (error) {
            *error = "DistanceEncoder::ComputeDistanceFeatures invalid state";
        }
        return false;
    }

    const int point_count = static_cast<int>(points_in_robot.cols());
    const int edge_count = static_cast<int>(halfspace_offset_.size());
    if (point_count <= 0) {
        features->min_distance = std::numeric_limits<double>::infinity();
        features->nearest_dual_direction.setZero();
        return true;
    }

    const size_t input_size = static_cast<size_t>(point_count) * 2u;
    input_buffer_.resize(input_size);
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
    if (output_it == outputs_.end() || output_it->second.empty()) {
        if (error) {
            *error = "DistanceEncoder: missing output tensor " + output_name_;
        }
        return false;
    }
    if (static_cast<int>(output_it->second.size()) < point_count * edge_count) {
        if (error) {
            *error = "DistanceEncoder: unexpected output size";
        }
        return false;
    }

    dual_coefficients_.resize(edge_count, point_count);
    const float* mu_data = output_it->second.data();
    for (int i = 0; i < point_count; ++i) {
        const float* row =
            mu_data + static_cast<size_t>(i) * static_cast<size_t>(edge_count);
        for (int edge = 0; edge < edge_count; ++edge) {
            dual_coefficients_(edge, i) = static_cast<double>(row[edge]);
        }
    }

    residual_matrix_.noalias() = halfspace_matrix_ * points_in_robot;
    residual_matrix_.colwise() -= halfspace_offset_;
    distances_.resize(point_count);
    distances_.noalias() = (dual_coefficients_.cwiseProduct(residual_matrix_))
                               .colwise()
                               .sum()
                               .transpose();

    int nearest_index = 0;
    double min_distance = distances_(0);
    for (int i = 1; i < point_count; ++i) {
        if (distances_(i) < min_distance) {
            min_distance = distances_(i);
            nearest_index = i;
        }
    }

    // lambda = -R A^T mu  for the nearest column only.
    features->nearest_dual_direction.noalias() =
        -(world_from_robot_rotation * halfspace_matrix_AT_ *
          dual_coefficients_.col(nearest_index));
    features->min_distance = min_distance;
    return true;
}

}  // namespace neupan_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
