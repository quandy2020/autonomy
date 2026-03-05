/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#pragma once

#include <cmath>
#include <iostream>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "Eigen/Dense"
#include "autonomy/common/math/hermite_spline.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/map/costmap_2d/costmap_2d.hpp"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/planning/common/smoother_exceptions.hpp"
#include "autonomy/planning/common/smoother_interface.hpp"

namespace autonomy {
namespace planning {
namespace smoother {

/**
 * @class autonomy::planning::smoother::SavitzkyGolaySmoother
 * @brief A path smoother implementation using Savitzky Golay filters
 */
class SavitzkyGolaySmoother : public common::Smoother {
 public:
  /**
   * @brief A constructor for
   * autonomy::planning::smoother::SavitzkyGolaySmoother
   */
  SavitzkyGolaySmoother() = default;

  /**
   * @brief A destructor for
   * autonomy::planning::smoother::SavitzkyGolaySmoother
   */
  ~SavitzkyGolaySmoother() override = default;

  void Configure(std::string name, std::shared_ptr<void> /*costmap_sub*/,
                 std::shared_ptr<map::costmap_2d::Costmap2DWrapper> /*costmap_wrapper*/) override;

  /**
   * @brief Method to cleanup resources.
   */
  void Cleanup() override {}

  /**
   * @brief Method to activate smoother and any threads involved in execution.
   */
  void Activate() override {}

  /**
   * @brief Method to deactivate smoother and any threads involved in
   * execution.
   */
  void Deactivate() override {}

  /**
   * @brief Method to smooth given path
   *
   * @param path In-out path to be smoothed
   * @param max_time Maximum duration smoothing should take
   * @return If smoothing was completed (true) or interrupted by time limit
   * (false)
   */
  bool Smooth(commsgs::planning_msgs::Path& path, const std::chrono::milliseconds& max_time) override;

  /**
   * @brief Method to calculate SavitzkyGolay Coefficients
   */
  void CalculateCoefficients();

 protected:
  /**
   * @brief Smoother method - does the smoothing on a segment
   * @param path Reference to path
   * @param reversing_segment Return if this is a reversing segment
   * @param costmap Pointer to minimal costmap
   * @param max_time Maximum time to compute, stop early if over limit
   * @return If smoothing was successful
   */
  bool SmoothImpl(commsgs::planning_msgs::Path& path, bool& reversing_segment);

  bool do_refinement_, enforce_path_inversion_;
  int refinement_num_, window_size_, half_window_size_, poly_order_;
  Eigen::VectorXd sg_coeffs_;
};

}  // namespace smoother
}  // namespace planning
}  // namespace autonomy