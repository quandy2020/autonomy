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

#include <algorithm>

#include <bolza/mpc/mpc_base.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MPCBase::MPCBase(mpc::Settings mpcSettings) : mpcSettings_(std::move(mpcSettings)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPCBase::Reset() {
  initRun_ = true;
  mpcTimer_.Reset();
  GetSolverPtr()->Reset();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool MPCBase::Run(scalar_t currentTime, const vector_t& currentState) {
  // check if the current time exceeds the solver final limit
  if (!initRun_ && currentTime >= GetSolverPtr()->GetFinalTime()) {
    std::cerr << "WARNING: The MPC time-horizon is smaller than the MPC starting time.\n";
    std::cerr << "currentTime: " << currentTime << "\t Controller finalTime: " << GetSolverPtr()->GetFinalTime() << '\n';
    return false;
  }

  const scalar_t finalTime = currentTime + mpcSettings_.timeHorizon_;

  // display
  if (mpcSettings_.debugPrint_) {
    std::cerr << "\n#####################################################";
    std::cerr << "\n#####################################################";
    std::cerr << "\n#####################################################";
    std::cerr << "\n### MPC is called at time:  " << currentTime << " [s].";
    std::cerr << "\n### MPC final Time:         " << finalTime << " [s].";
    std::cerr << "\n### MPC time horizon:       " << mpcSettings_.timeHorizon_ << " [s].\n";
    mpcTimer_.StartTimer();
  }

  // calculate the MPC policy
  CalculateController(currentTime, currentState, finalTime);

  // set initRun flag to false
  initRun_ = false;

  // display
  if (mpcSettings_.debugPrint_) {
    mpcTimer_.EndTimer();
    std::cerr << "\n### MPC Benchmarking";
    std::cerr << "\n###   Maximum : " << mpcTimer_.GetMaxIntervalInMilliseconds() << "[ms].";
    std::cerr << "\n###   Average : " << mpcTimer_.GetAverageInMilliseconds() << "[ms].";
    std::cerr << "\n###   Latest  : " << mpcTimer_.GetLastIntervalInMilliseconds() << "[ms]." << std::endl;
  }

  return true;
}

}  // namespace bolza
