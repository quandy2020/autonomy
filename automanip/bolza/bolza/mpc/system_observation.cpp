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

#include "bolza/mpc/system_observation.hpp"

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void swap(SystemObservation& a, SystemObservation& b) noexcept {
  std::swap(a.mode, b.mode);
  std::swap(a.time, b.time);
  a.state.swap(b.state);
  a.input.swap(b.input);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::ostream& operator<<(std::ostream& out, const SystemObservation& observation) {
  Out << "Observation: \n";
  Out << "\t time:      " << observation.time << '\n';
  Out << "\t mode:      " << observation.mode << '\n';
  Out << "\t state:    [";
  for (int i = 0; i < observation.state.size() - 1; i++) {
    Out << observation.state(i) << ", ";
  }
  Out << observation.state(observation.state.size() - 1) << "]\n";
  Out << "\t input:    [";
  for (int i = 0; i < observation.input.size() - 1; i++) {
    Out << observation.input(i) << ", ";
  }
  Out << observation.input(observation.input.size() - 1) << "]";

  return out;
}

}  // namespace bolza
