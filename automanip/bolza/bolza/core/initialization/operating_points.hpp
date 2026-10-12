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
 * @file operating_points.hpp
 * @brief This is an implementation of Initializer class that uses operating trajectories or single point for state-input initialization.
 */


#include <bolza/core/initialization/initializer.hpp>
#include <bolza/core/misc/linear_interpolation.hpp>

namespace bolza {

/**
 * This is an implementation of Initializer class that uses operating trajectories or single point for state-input initialization.
 */
class OperatingPoints final : public Initializer {
 public:
  /**
   * Constructor
   * @param [in] stateOperatingPoint: An operating point for state.
   * @param [in] inputOperatingPoint: An operating point for input.
   */
  OperatingPoints(const vector_t& stateOperatingPoint, const vector_t& inputOperatingPoint)
      /**
       * @brief Time Trajectory.
       * @param [in] inputOperatingPoint: Input Operating Point.
       * @return Result value.
       */
      : timeTrajectory_(1, 0.0), stateTrajectory_(1, stateOperatingPoint), inputTrajectory_(1, inputOperatingPoint) {}

  /**
   * Constructor
   * @param [in] timeTrajectory: The time stamps of the operating trajectories.
   * @param [in] stateTrajectory: The state operating trajectory.
   * @param [in] inputTrajectory: The input operating trajectory.
   */
  OperatingPoints(scalar_array_t timeTrajectory, vector_array_t stateTrajectory, vector_array_t inputTrajectory)
      /**
       * @brief Time Trajectory.
       * @return Result value.
       */
      : timeTrajectory_(std::move(timeTrajectory)),
        stateTrajectory_(std::move(stateTrajectory)),
        inputTrajectory_(std::move(inputTrajectory)) {}

  /** Destructor */
  ~OperatingPoints() override = default;

  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  OperatingPoints* Clone() const override { /**
  * @brief Operating Points.
  * @param [in,out] this: This.
  * @return Result value.
  */
 return new OperatingPoints(*this); }

  /**
   * @brief Compute.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] nextTime: Next Time.
   * @param [in,out] input: Input.
   * @param [in,out] nextState: Next State.
   */
  void compute(scalar_t time, const vector_t& state, scalar_t nextTime, vector_t& input, vector_t& nextState) override {
    /**
     * @brief Interpolate.
     * @param [in] time: Time.
     * @param [in] timeTrajectory_: Time Trajectory.
     * @param [in] inputTrajectory_: Input Trajectory.
     * @return Result value.
     */
    input = LinearInterpolation::Interpolate(time, timeTrajectory_, inputTrajectory_);
    /**
     * @brief Interpolate.
     * @param [in] nextTime: Next Time.
     * @param [in] timeTrajectory_: Time Trajectory.
     * @param [in] stateTrajectory_: State Trajectory.
     * @return Result value.
     */
    nextState = LinearInterpolation::Interpolate(nextTime, timeTrajectory_, stateTrajectory_);
  }

 private:
    /**
   * @brief Copy constructor 
   * @param [in] other: Other.
   */
  OperatingPoints(const OperatingPoints& other) = default;

  const scalar_array_t timeTrajectory_;
  const vector_array_t stateTrajectory_;
  const vector_array_t inputTrajectory_;
};

}  // namespace bolza
