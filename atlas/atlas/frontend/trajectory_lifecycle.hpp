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

#ifndef ATLAS_FRONTEND_TRAJECTORY_LIFECYCLE_HPP_
#define ATLAS_FRONTEND_TRAJECTORY_LIFECYCLE_HPP_

namespace atlas {
namespace frontend {

/** @brief Lifecycle states for one trajectory. */
enum class TrajectoryLifecycleState {
  kStopped,
  kAwaitingInitialization,
  kTracking,
  kDegraded,
  kFinalizing,
};

/** @brief Owns valid trajectory lifecycle transitions. */
class TrajectoryLifecycle {
 public:
  TrajectoryLifecycleState value() const { return value_; }

  bool Start() {
    if (value_ != TrajectoryLifecycleState::kStopped) {
      return false;
    }
    value_ = TrajectoryLifecycleState::kAwaitingInitialization;
    return true;
  }

  bool InitializationSucceeded() {
    if (value_ != TrajectoryLifecycleState::kAwaitingInitialization &&
        value_ != TrajectoryLifecycleState::kDegraded) {
      return false;
    }
    value_ = TrajectoryLifecycleState::kTracking;
    return true;
  }

  bool MarkDegraded() {
    if (value_ != TrajectoryLifecycleState::kTracking &&
        value_ != TrajectoryLifecycleState::kAwaitingInitialization) {
      return false;
    }
    value_ = TrajectoryLifecycleState::kDegraded;
    return true;
  }

  bool BeginFinalization() {
    if (value_ == TrajectoryLifecycleState::kStopped ||
        value_ == TrajectoryLifecycleState::kFinalizing) {
      return false;
    }
    value_ = TrajectoryLifecycleState::kFinalizing;
    return true;
  }

  void Stop() { value_ = TrajectoryLifecycleState::kStopped; }

 private:
  TrajectoryLifecycleState value_ = TrajectoryLifecycleState::kStopped;
};

}  // namespace frontend
}  // namespace atlas

#endif  // ATLAS_FRONTEND_TRAJECTORY_LIFECYCLE_HPP_
