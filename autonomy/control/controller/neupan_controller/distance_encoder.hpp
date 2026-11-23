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

#pragma once

/**
 * @file distance_encoder.hpp
 * @brief ONNX ObsPointNet: robot-frame points → dual coefficients mu.
 */

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "autonomy/common/network/network.hpp"
#include "autonomy/control/controller/neupan_controller/types.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace neupan_controller {

class DistanceEncoder
{
public:
    static std::unique_ptr<DistanceEncoder> Create(
        const std::string& model_path, const Mat& halfspace_G, const Vec& halfspace_h,
        const std::string& inference_backend,
        const std::string& inference_execution_provider, std::string* error);

    bool InferDuals(const Mat2X& points_in_robot, Mat* mu, Vec* distances,
                    std::string* error) const;

    int GetEdgeCount() const {
        return static_cast<int>(halfspace_offset_.size());
    }

private:
    DistanceEncoder(const Mat& G, const Vec& h,
                    std::unique_ptr<::autonomy::common::network::Engine> engine,
                    std::string input_name, std::string output_name);

    Mat halfspace_matrix_;
    Vec halfspace_offset_;
    std::unique_ptr<::autonomy::common::network::Engine> engine_;
    std::string input_name_;
    std::string output_name_;
    mutable std::mutex inference_mutex_;
    mutable std::vector<float> input_buffer_;
    mutable ::autonomy::common::network::FloatTensorMap inputs_;
    mutable ::autonomy::common::network::FloatTensorMap outputs_;
};

}  // namespace neupan_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
