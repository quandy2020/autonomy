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
 * @file trajectory_spreading.hpp
 * @brief Constructor.
 */


#include <bolza/core/types.hpp>
#include <bolza/core/reference/mode_schedule.hpp>

namespace bolza {

class TrajectorySpreading final {
 public:
  struct Status {
    bool willTruncate = false;
    bool willPerformTrajectorySpreading = false;
  };

  /**
   * @brief Constructor.
   * @param [in] debugPrint: Debug Print.
   */
  explicit TrajectorySpreading(bool debugPrint = false) : debugPrint_(debugPrint) {}

  /**
   * Initialize trajectory spreading strategy. Run it before invoking other member functions
   *
   * @param [in] oldModeSchedule: The old mode schedule associated to the trajectories which should be adjusted.
   * @param [in] newModeSchedule: The new mode schedule that should be adapted to.
   * @param [in] oldTimeTrajectory: The old time trajectories that is associated with the old mode schedule.
   * @returns the status of the devised trajectory spreading strategy.
   */
  Status Set(const ModeSchedule& oldModeSchedule, const ModeSchedule& newModeSchedule, const scalar_array_t& oldTimeTrajectory);

  /**
   * @brief Gets the trajectory-spreading strategy report.
   * @return Result value.
   */
  const Status& GetStatus() const { return status_; }

  /**
   * Adjust continuous-time trajectory.
   *
   * @tparam data type.
   * @param [in,out] trajectory: trajectory for rectification.
   */
  template <typename T>
  /**
   * @brief Adjust Trajectory.
   * @param [in,out] trajectory: Trajectory.
   */
  void AdjustTrajectory(std::vector<T>& trajectory) const;

  /**
   * Extracts event-time data.
   *
   * @tparam data type.
   * @param [in] array: The input array for rectification.
   * @return The output rectified array.
   */
  template <typename T>
  /**
   * @brief Extract Events Array.
   * @param [in] array: Array.
   * @return Result value.
   */
  std::vector<T> ExtractEventsArray(const std::vector<T>& array) const;

  /**
   * Adjust time stamp of the trajectories and post event indices of the trajectories.
   *
   * @param [in,out] timeTrajectory: The time stamp of the trajectories.
   */
  void AdjustTimeTrajectory(scalar_array_t& timeTrajectory) const;

  /**
   * Get adjusted post event index array.
   *
   * @return The post event indices of the trajectories.
   */
  const size_array_t& GetPostEventIndices() const { return updatedPostEventIndices_; }

 private:
  /**
   * Computing spreading strategy based on the correspondence between the old event time and the new event time.
   *
   * @param [in] oldTimeTrajectory: The old time trajectories that is associated with the old mode schedule.
   * @param [in] oldMatchedEventTimes: The old event time that need adjustment potentially.
   * @param [in] newMatchedEventTimes: The corresponding new event time.
   */
  void ComputeSpreadingStrategy(const scalar_array_t& oldTimeTrajectory, const scalar_array_t& oldMatchedEventTimes,
                                const scalar_array_t& newMatchedEventTimes);

  // helper function: get index of the iterator returned by upper_bound function
  /**
   * @brief Upper Bound Index.
   * @param [in] timeTrajectory: Time Trajectory.
   * @param [in] queryTime: Query Time.
   * @return Result value.
   */
  int UpperBoundIndex(const scalar_array_t& timeTrajectory, scalar_t queryTime) const {
    /**
     * @brief Distance.
     * @return Result value.
     */
    return std::distance(timeTrajectory.begin(), std::upper_bound(timeTrajectory.begin(), timeTrajectory.end(), queryTime));
  }

  // helper function: get index of the iterator returned by lower_bound function
  /**
   * @brief Lower Bound Index.
   * @param [in] timeTrajectory: Time Trajectory.
   * @param [in] queryTime: Query Time.
   * @return Result value.
   */
  int LowerBoundIndex(const scalar_array_t& timeTrajectory, scalar_t queryTime) const {
    /**
     * @brief Distance.
     * @return Result value.
     */
    return std::distance(timeTrajectory.begin(), std::lower_bound(timeTrajectory.begin(), timeTrajectory.end(), queryTime));
  }

  /**
   * Finds post event indices of an array of event time for the given time trajectory.
   *
   * @param [in] eventTimes: An array of event time.
   * @param [in] timeTrajectory: Time trajectory.
   * @return An array containing post event indices of the given event time.
   */
  size_array_t FindPostEventIndices(const scalar_array_t& eventTimes, const scalar_array_t& timeTrajectory) const {
    /**
     * @brief Post Event Indices.
     * @return Result value.
     */
    size_array_t postEventIndices(eventTimes.size());
    for (std::size_t i = 0; i < eventTimes.size(); i++) {
      if (i == eventTimes.size() - 1 && eventTimes[i] == timeTrajectory.back()) {
        postEventIndices[i] = timeTrajectory.size() - 1;
      } else {
        postEventIndices[i] = UpperBoundIndex(timeTrajectory, eventTimes[i]);
      }
    }
    return postEventIndices;
  }

  /***********
   * Variables
   ***********/
  const bool debugPrint_;

  Status status_;
  size_t eraseFromIndex_;                             /**< The first index to erase **/
  std::pair<size_t, size_t> keepEventDataInInterval_; /**< Keep event data within [first, second) **/

  size_array_t beginIndices_;
  size_array_t endIndices_;
  size_array_t spreadingValueIndices_;

  size_array_t updatedPostEventIndices_;
  scalar_array_t updatedMatchedEventTimes_;
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename T>
/**
 * @brief Extract Events Array.
 * @param [in] array: Array.
 * @return Result value.
 */
std::vector<T> TrajectorySpreading::ExtractEventsArray(const std::vector<T>& array) const {
  const auto firstItr = array.begin() + keepEventDataInInterval_.first;
  const auto lastItr = array.begin() + keepEventDataInInterval_.second;
  /**
   * @brief Out.
   * @param [in] firstItr: First Itr.
   * @param [in] lastItr: Last Itr.
   * @return Result value.
   */
  std::vector<T> out(firstItr, lastItr);
  return out;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename T>
/**
 * @brief Adjust Trajectory.
 * @param [in,out] trajectory: Trajectory.
 * @return Result value.
 */
void TrajectorySpreading::AdjustTrajectory(std::vector<T>& trajectory) const {
  // erase segment of trajectory associated to mismatched modes
  /**
   * @brief Erase.
   * @param [in] eraseFromIndex_: Erase From Index.
   * @return Result value.
   */
  trajectory.erase(trajectory.begin() + eraseFromIndex_, trajectory.end());

  // extract spreading values beforehand since they might be overridden
  /**
   * @brief Spreading Values.
   * @return Result value.
   */
  std::vector<T> spreadingValues(spreadingValueIndices_.size());
  /**
   * @brief Transform.
   * @return Result value.
   */
  std::transform(spreadingValueIndices_.begin(), spreadingValueIndices_.end(), spreadingValues.begin(),
                 [&](const size_t& ind) { return trajectory[ind]; });

  // spread
  for (size_t i = 0; i < spreadingValueIndices_.size(); i++) {
    for (size_t j = beginIndices_[i]; j < endIndices_[i]; j++) {
      trajectory[j] = spreadingValues[i];
    }  // end of i loop
  }    // end of j loop
}

}  // namespace bolza
