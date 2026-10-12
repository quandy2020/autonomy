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

#include "bolza/mpc/mpc_mrt_interface.hpp"

#include <bolza/core/control/feedforward_controller.hpp>
#include <bolza/core/control/linear_controller.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MPCMRTInterface::MPCMRTInterface(MPCBase& mpc) : mpc_(mpc) {
  mpcTimer_.reset();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPCMRTInterface::ResetMpcNode(const TargetTrajectories& initTargetTrajectories) {
  mpc_.reset();
  mpc_.getSolverPtr()->getReferenceManager().setTargetTrajectories(initTargetTrajectories);
  mpcTimer_.reset();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPCMRTInterface::SetCurrentObservation(const SystemObservation& currentObservation) {
  std::lock_guard<std::mutex> lock(observationMutex_);
  currentObservation_ = currentObservation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ReferenceManagerInterface& MPCMRTInterface::GetReferenceManager() {
  return mpc_.getSolverPtr()->getReferenceManager();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
const ReferenceManagerInterface& MPCMRTInterface::GetReferenceManager() const {
  return mpc_.getSolverPtr()->getReferenceManager();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPCMRTInterface::AdvanceMpc() {
  // measure the delay in running MPC
  mpcTimer_.startTimer();

  SystemObservation currentObservation;
  {
    std::lock_guard<std::mutex> lock(observationMutex_);
    currentObservation = currentObservation_;
  }

  bool controllerIsUpdated = mpc_.run(currentObservation.time, currentObservation.state);
  if (!controllerIsUpdated) {
    return;
  }
  CopyToBuffer(currentObservation);

  // measure the delay for sending ROS messages
  mpcTimer_.endTimer();

  // check MPC delay and solution window compatibility
  scalar_t timeWindow = mpc_.settings().solutionTimeWindow_;
  if (mpc_.settings().solutionTimeWindow_ < 0) {
    timeWindow = mpc_.getSolverPtr()->getFinalTime() - currentObservation.time;
  }
  if (timeWindow < 2.0 * mpcTimer_.getAverageInMilliseconds() * 1e-3) {
    std::cerr << "[MPCMRTInterface::AdvanceMpc] WARNING: The solution time window might be shorter than the MPC delay!\n";
  }

  // measure the delay
  if (mpc_.settings().debugPrint_) {
    std::cerr << "\n### MPC_MRT Benchmarking";
    std::cerr << "\n###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
    std::cerr << "\n###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms].";
    std::cerr << "\n###   Latest  : " << mpcTimer_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPCMRTInterface::CopyToBuffer(const SystemObservation& mpcInitObservation) {
  // policy
  auto primalSolutionPtr = std::make_unique<PrimalSolution>();
  const scalar_t startTime = mpcInitObservation.time;
  const scalar_t finalTime =
      (mpc_.settings().solutionTimeWindow_ < 0) ? mpc_.getSolverPtr()->getFinalTime() : startTime + mpc_.settings().solutionTimeWindow_;
  mpc_.getSolverPtr()->getPrimalSolution(finalTime, primalSolutionPtr.get());

  // command
  auto commandPtr = std::make_unique<CommandData>();
  commandPtr->mpcInitObservation_ = mpcInitObservation;
  commandPtr->mpcTargetTrajectories_ = mpc_.getSolverPtr()->getReferenceManager().getTargetTrajectories();

  // performance indices
  auto performanceIndicesPtr = std::make_unique<PerformanceIndex>();
  *performanceIndicesPtr = mpc_.getSolverPtr()->getPerformanceIndeces();

  this->moveToBuffer(std::move(commandPtr), std::move(primalSolutionPtr), std::move(performanceIndicesPtr));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t MPCMRTInterface::GetLinearFeedbackGain(scalar_t time) {
  auto controller = dynamic_cast<LinearController*>(this->getPolicy().controllerPtr_.get());
  if (controller == nullptr) {
    throw std::runtime_error("[MPCMRTInterface::GetLinearFeedbackGain] Feedback gains only available with linear controller!");
  }
  matrix_t K;
  controller->getFeedbackGain(time, K);
  return K;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation MPCMRTInterface::GetValueFunction(scalar_t time, const vector_t& state) const {
  return mpc_.getSolverPtr()->getValueFunction(time, state);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t MPCMRTInterface::GetStateInputEqualityConstraintLagrangian(scalar_t time, const vector_t& state) const {
  return mpc_.getSolverPtr()->getStateInputEqualityConstraintLagrangian(time, state);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MultiplierCollection MPCMRTInterface::GetIntermediateDualSolution(scalar_t time) const {
  return mpc_.getSolverPtr()->getIntermediateDualSolution(time);
}

}  // namespace bolza
