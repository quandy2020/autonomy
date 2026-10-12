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

#include "bolza/core/reference/target_trajectories.hpp"

#include <bolza/core/misc/display.hpp>
#include <bolza/core/misc/linear_interpolation.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
TargetTrajectories::TargetTrajectories(size_t size) : timeTrajectory(size), stateTrajectory(size), inputTrajectory(size) {}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
TargetTrajectories::TargetTrajectories(scalar_array_t desiredTimeTrajectory, vector_array_t desiredStateTrajectory,
                                       vector_array_t desiredInputTrajectory)
    : timeTrajectory(std::move(desiredTimeTrajectory)),
      stateTrajectory(std::move(desiredStateTrajectory)),
      inputTrajectory(std::move(desiredInputTrajectory)) {
  assert(stateTrajectory.size() == timeTrajectory.size());
  if (!inputTrajectory.empty()) {
    assert(inputTrajectory.size() == timeTrajectory.size());
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
void TargetTrajectories::clear() {
  timeTrajectory.clear();
  stateTrajectory.clear();
  inputTrajectory.clear();
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
bool TargetTrajectories::operator==(const TargetTrajectories& other) {
  return this->timeTrajectory == other.timeTrajectory && this->stateTrajectory == other.stateTrajectory &&
         this->inputTrajectory == other.inputTrajectory;
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
vector_t TargetTrajectories::GetDesiredState(scalar_t time) const {
  if (this->empty()) {
    throw std::runtime_error("[TargetTrajectories] TargetTrajectories is empty!");
  } else {
    return LinearInterpolation::Interpolate(time, timeTrajectory, stateTrajectory);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
vector_t TargetTrajectories::GetDesiredInput(scalar_t time) const {
  if (this->empty()) {
    throw std::runtime_error("[TargetTrajectories] TargetTrajectories is empty!");
  } else if (inputTrajectory.empty()) {
    throw std::runtime_error("[TargetTrajectories] TargetTrajectories does not have inputTrajectory!");
  } else {
    return LinearInterpolation::Interpolate(time, timeTrajectory, inputTrajectory);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
void swap(TargetTrajectories& lh, TargetTrajectories& rh) {
  lh.timeTrajectory.swap(rh.timeTrajectory);
  lh.stateTrajectory.swap(rh.stateTrajectory);
  lh.inputTrajectory.swap(rh.inputTrajectory);
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
std::ostream& operator<<(std::ostream& out, const TargetTrajectories& targetTrajectories) {
  for (size_t i = 0; i < targetTrajectories.size(); i++) {
    out << "time: " << targetTrajectories.timeTrajectory[i] << "\n";
    out << "state: [" << ToDelimitedString(targetTrajectories.stateTrajectory[i]) << "]\n";
    out << "input: [" << ToDelimitedString(targetTrajectories.inputTrajectory[i]) << "]\n";
  }  // end of i loop

  return out;
}

}  // namespace bolza
