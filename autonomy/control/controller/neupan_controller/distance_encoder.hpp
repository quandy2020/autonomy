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

/**
 * @file distance_encoder.hpp
 * @brief ONNX ObsPointNet wrapper producing nearest-obstacle distance features.
 */

#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "autonomy/common/network/network.hpp"
#include "autonomy/control/controller/neupan_controller/robot_footprint.hpp"
#include "autonomy/control/controller/neupan_controller/types.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace neupan_controller {

/**
 * @class DistanceEncoder
 * @brief Loads and runs NeuPAN ObsPointNet via @c
 * autonomy::common::network::Engine.
 *
 * Accepts robot-frame points as Eigen [2, N]. Computes distances for all
 * points, then materializes dual direction only for the nearest column.
 */
class DistanceEncoder
{
public:
    static std::unique_ptr<DistanceEncoder> Create(
        const std::string& model_path, const RobotFootprint& footprint,
        const std::string& inference_backend,
        const std::string& inference_execution_provider, std::string* error);

    /**
     * @brief Infer nearest-obstacle distance features.
     *
     * @param[in] points_in_robot Obstacle points in the robot frame, shape [2,
     * N].
     * @param[in] world_from_robot_rotation 2x2 yaw rotation world <- robot.
     * @param[out] features Nearest-obstacle features.
     * @param[out] error Failure reason when returning false.
     * @return true on success.
     */
    bool ComputeDistanceFeatures(
        const Eigen::Matrix2Xd& points_in_robot,
        const Eigen::Matrix2d& world_from_robot_rotation,
        DistanceFeatures* features, std::string* error) const;

    int GetEdgeCount() const {
        return static_cast<int>(halfspace_offset_.size());
    }

private:
    DistanceEncoder(const RobotFootprint& footprint,
                    std::unique_ptr<common::network::Engine> engine,
                    std::string input_name, std::string output_name);

    Eigen::MatrixXd halfspace_matrix_;     // edge x 2
    Eigen::MatrixXd halfspace_matrix_AT_;  // 2 x edge
    Eigen::VectorXd halfspace_offset_;     // edge

    std::unique_ptr<common::network::Engine> engine_;
    std::string input_name_;
    std::string output_name_;
    mutable std::mutex inference_mutex_;

    // Hot-path scratch (reused).
    mutable std::vector<float> input_buffer_;
    mutable Eigen::MatrixXd dual_coefficients_;  // edge x N
    mutable Eigen::MatrixXd residual_matrix_;    // edge x N
    mutable Eigen::VectorXd distances_;          // N
    mutable common::network::FloatTensorMap inputs_;
    mutable common::network::FloatTensorMap outputs_;
};

}  // namespace neupan_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
