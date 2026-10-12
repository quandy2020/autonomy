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

#include "bolza/oc/multiple_shooting/initialization.hpp"

namespace bolza {
namespace multiple_shooting {

void InitializeStateInputTrajectories(const vector_t& initState, const std::vector<AnnotatedTime>& timeDiscretization,
                                      const PrimalSolution& primalSolution, Initializer& initializer, vector_array_t& stateTrajectory,
                                      vector_array_t& inputTrajectory) {
  const int N = static_cast<int>(timeDiscretization.size()) - 1;  // // size of the input trajectory
  stateTrajectory.clear();
  stateTrajectory.reserve(N + 1);
  inputTrajectory.clear();
  inputTrajectory.reserve(N);

  // Determine till when to use the previous solution
  scalar_t interpolateStateTill = timeDiscretization.front().time;
  scalar_t interpolateInputTill = timeDiscretization.front().time;
  if (primalSolution.timeTrajectory_.size() >= 2) {
    interpolateStateTill = primalSolution.timeTrajectory_.back();
    interpolateInputTill = primalSolution.timeTrajectory_[primalSolution.timeTrajectory_.size() - 2];
  }

  // Initial state
  const scalar_t initTime = GetIntervalStart(timeDiscretization[0]);
  if (initTime < interpolateStateTill) {
    stateTrajectory.push_back(LinearInterpolation::Interpolate(initTime, primalSolution.timeTrajectory_, primalSolution.stateTrajectory_));
  } else {
    stateTrajectory.push_back(initState);
  }

  for (int i = 0; i < N; i++) {
    if (timeDiscretization[i].event == AnnotatedTime::Event::PreEvent) {
      // Event Node
      inputTrajectory.push_back(vector_t());  // no input at event node
      stateTrajectory.push_back(InitializeEventNode(timeDiscretization[i].time, stateTrajectory.back()));
    } else {
      // Intermediate node
      const scalar_t time = GetIntervalStart(timeDiscretization[i]);
      const scalar_t nextTime = GetIntervalEnd(timeDiscretization[i + 1]);
      vector_t input, nextState;
      if (time > interpolateInputTill || nextTime > interpolateStateTill) {  // Using initializer
        std::tie(input, nextState) = InitializeIntermediateNode(initializer, time, nextTime, stateTrajectory.back());
      } else {  // interpolate previous solution
        std::tie(input, nextState) = InitializeIntermediateNode(primalSolution, time, nextTime);
      }
      inputTrajectory.push_back(std::move(input));
      stateTrajectory.push_back(std::move(nextState));
    }
  }
}

}  // namespace multiple_shooting
}  // namespace bolza
