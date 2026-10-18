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

/**
 * @file degeneracy_detector.hpp
 * @brief FIM eigenvalue degeneracy (tunnel / corridor). See docs/05_frontend_fusion.md.
 */

#ifndef ARGUS_PERCEPTION_DEGENERACY_DETECTOR_HPP_
#define ARGUS_PERCEPTION_DEGENERACY_DETECTOR_HPP_

#include <Eigen/Core>

namespace argus {
namespace perception {

/**
 * @brief Thresholds for Fisher information matrix (FIM) degeneracy tests.
 */
struct DegeneracyConfig {
    /** @brief Min eigenvalue before a direction is treated as weak. */
    double eigenvalue_threshold = 0.05;
    /** @brief Max λ_max / λ_min before the FIM is ill-conditioned. */
    double condition_number_threshold = 100.0;
    /** @brief Inflate state covariance along weak directions. */
    bool enable_anisotropic_inflation = true;
    /** @brief Covariance inflation factor α. */
    double inflation_scale = 10.0;
    /** @brief Analyze the xyz (position) block of the FIM only. */
    bool position_block_only = true;
};

/**
 * @brief Result of one FIM degeneracy analysis.
 */
struct DegeneracyResult {
    /** @brief True if weak observability was detected. */
    bool is_degenerated = false;
    /** @brief Unit vector of the weakest observability axis. */
    Eigen::Vector3d weak_directions = Eigen::Vector3d::Zero();
    /** @brief Eigenvalues of the analyzed FIM block (ascending). */
    Eigen::Vector3d eigenvalues = Eigen::Vector3d::Zero();
    /** @brief λ_max / λ_min. */
    double condition_number = 1.0;
};

/**
 * @brief Detects poorly observable directions from a LiDAR FIM.
 */
class DegeneracyDetector
{
public:
    /**
     * @brief Construct with analysis thresholds.
     * @param config Eigenvalue / condition thresholds and inflation policy.
     */
    explicit DegeneracyDetector(DegeneracyConfig config = {});

    /**
     * @brief Analyze a Fisher information matrix.
     * @param fim Square FIM (full state or position block per config).
     * @return Degeneracy flags, eigenvalues, and weak directions.
     */
    DegeneracyResult Analyze(const Eigen::MatrixXd& fim) const;

    /** @return Current configuration. */
    const DegeneracyConfig& config() const {
        return config_;
    }

    /**
     * @brief Replace configuration.
     * @param config New thresholds / inflation policy.
     */
    void SetConfig(const DegeneracyConfig& config) {
        config_ = config;
    }

private:
    DegeneracyConfig config_;
};

}  // namespace perception
}  // namespace argus

#endif  // ARGUS_PERCEPTION_DEGENERACY_DETECTOR_HPP_
