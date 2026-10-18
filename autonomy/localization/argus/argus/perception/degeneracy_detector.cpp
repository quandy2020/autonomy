/*
 * Copyright 2026 The Openbot Authors
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

#include "argus/perception/degeneracy_detector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include <Eigen/Eigenvalues>

namespace argus {
namespace perception {

DegeneracyDetector::DegeneracyDetector(DegeneracyConfig config)
    : config_(std::move(config)) {}

DegeneracyResult DegeneracyDetector::Analyze(const Eigen::MatrixXd& fim) const {
    DegeneracyResult result;
    if (fim.rows() < 3 || fim.cols() < 3) {
        return result;
    }

    // Position-block analysis (xyz). Rotation-inclusive path reserved.
    (void)config_.position_block_only;
    const Eigen::Matrix3d position_fim = fim.topLeftCorner<3, 3>();

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(position_fim);
    if (solver.info() != Eigen::Success) {
        return result;
    }

    result.eigenvalues = solver.eigenvalues();
    const double min_eigenvalue = result.eigenvalues.minCoeff();
    const double max_eigenvalue = result.eigenvalues.maxCoeff();
    result.condition_number =
        (min_eigenvalue > 1e-12)
            ? (max_eigenvalue / min_eigenvalue)
            : std::numeric_limits<double>::infinity();

    Eigen::Index weak_index = 0;
    result.eigenvalues.minCoeff(&weak_index);
    result.weak_directions =
        solver.eigenvectors().col(weak_index).normalized();

    result.is_degenerated =
        (min_eigenvalue < config_.eigenvalue_threshold) ||
        (result.condition_number > config_.condition_number_threshold);
    return result;
}

}  // namespace perception
}  // namespace argus
