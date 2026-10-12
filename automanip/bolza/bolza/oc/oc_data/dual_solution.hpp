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
 * @file dual_solution.hpp
 * @brief Exchanges the content of DualSolution.
 */


#include <bolza/core/types.hpp>
#include <bolza/core/model_data/multiplier.hpp>

namespace bolza {

struct DualSolution {
  scalar_array_t timeTrajectory;
  size_array_t postEventIndices;

  MultiplierCollection final;
  std::vector<MultiplierCollection> preJumps;
  std::vector<MultiplierCollection> intermediates;

  /**
   * @brief Exchanges the content of DualSolution.
   * @param [in,out] other: Other.
   */
  void swap(DualSolution& other) {
    /**
     * @brief Swap.
     * @param [in] timeTrajectory: Time Trajectory.
     * @return Result value.
     */
    timeTrajectory.swap(other.timeTrajectory);
    /**
     * @brief Swap.
     * @param [in] postEventIndices: Post Event Indices.
     * @return Result value.
     */
    postEventIndices.swap(other.postEventIndices);
    /**
     * @brief Swap.
     * @param [in] final: Final.
     * @return Result value.
     */
    final.swap(other.final);
    /**
     * @brief Swap.
     * @param [in] preJumps: Pre Jumps.
     * @return Result value.
     */
    preJumps.swap(other.preJumps);
    /**
     * @brief Swap.
     * @param [in] intermediates: Intermediates.
     * @return Result value.
     */
    intermediates.swap(other.intermediates);
  }

  /** Clears the content of the DualSolution */
  void clear() {
    /**
     * @brief Clear.
     * @return Result value.
     */
    timeTrajectory.clear();
    /**
     * @brief Clear.
     * @return Result value.
     */
    postEventIndices.clear();
    /**
     * @brief Clear.
     * @return Result value.
     */
    final.clear();
    /**
     * @brief Clear.
     * @return Result value.
     */
    preJumps.clear();
    /**
     * @brief Clear.
     * @return Result value.
     */
    intermediates.clear();
  }
};

/**
 * Note that this view of DualSolution does not have access to its timestamp, signaling that functions
 * operating on it cannot modify its timestamp.
 */
struct DualSolutionRef {
  /**
   * @brief Constructs a DualSolutionRef.
   * @param [in,out] intermediates: Intermediates.
   */
  DualSolutionRef(DualSolution& dualSolution) : DualSolutionRef(dualSolution.final, dualSolution.preJumps, dualSolution.intermediates) {}

  DualSolutionRef(MultiplierCollection& finalRef, std::vector<MultiplierCollection>& preJumpsRef,
                  std::vector<MultiplierCollection>& intermediatesRef)
      /**
       * @brief Final.
       * @param [in] intermediatesRef: Intermediates Ref.
       * @return Result value.
       */
      : final(finalRef), preJumps(preJumpsRef), intermediates(intermediatesRef) {}

  MultiplierCollection& final;
  std::vector<MultiplierCollection>& preJumps;
  std::vector<MultiplierCollection>& intermediates;
};

/**
 * Note that this view of DualSolution does not have access to its timestamp, signaling that functions
 * operating on it are not depending on the timestamp.
 */
struct DualSolutionConstRef {
  DualSolutionConstRef(const DualSolution& dualSolution)
      /**
       * @brief Dual Solution Const Ref.
       * @param [in] final: Final.
       * @param [in] preJumps: Pre Jumps.
       * @param [in] intermediates: Intermediates.
       * @return Result value.
       */
      : DualSolutionConstRef(dualSolution.final, dualSolution.preJumps, dualSolution.intermediates) {}

  DualSolutionConstRef(const MultiplierCollection& finalRef, const std::vector<MultiplierCollection>& preJumpsRef,
                       const std::vector<MultiplierCollection>& intermediatesRef)
      /**
       * @brief Final.
       * @param [in] intermediatesRef: Intermediates Ref.
       * @return Result value.
       */
      : final(finalRef), preJumps(preJumpsRef), intermediates(intermediatesRef) {}

  const MultiplierCollection& final;
  const std::vector<MultiplierCollection>& preJumps;
  const std::vector<MultiplierCollection>& intermediates;
};

/**
 * Calculates the intermediate dual solution at the given time.
 *
 * @param [in] dualSolution: The dual solution
 * @param [in] time: The inquiry time
 * @return The collection of multipliers associated to state/state-input, equality/inequality Lagrangian terms.
 */
inline MultiplierCollection GetIntermediateDualSolutionAtTime(const DualSolution& dualSolution, scalar_t time) {
  const auto indexAlpha = LinearInterpolation::TimeSegment(time, dualSolution.timeTrajectory);
  /**
   * @brief Interpolate.
   * @param [in] indexAlpha: Index Alpha.
   * @param [in] intermediates: Intermediates.
   * @return Result value.
   */
  return LinearInterpolation::Interpolate(indexAlpha, dualSolution.intermediates);
}

/**
 * Samples the intermediate dual solution at the given time array.
 *
 * @param [in] dualSolution: The dual solution
 * @param [in] timeTrajectory: The time array
 * @param [out] intermediateDualSolution: An array of sampled MultiplierCollection.
 */
inline void SampleIntermediateDualSolution(const DualSolution& dualSolution, const scalar_array_t& timeTrajectory,
                                           std::vector<MultiplierCollection>& intermediateDualSolution) {
  // re-sample dual solution
  /**
   * @brief Clear.
   * @return Result value.
   */
  intermediateDualSolution.clear();
  /**
   * @brief Reserve.
   * @return Result value.
   */
  intermediateDualSolution.reserve(timeTrajectory.size());
  for (const auto& t : timeTrajectory) {
    /**
     * @brief Push back.
     * @return Result value.
     */
    intermediateDualSolution.push_back(GetIntermediateDualSolutionAtTime(dualSolution, t));
  }
}

}  // namespace bolza
