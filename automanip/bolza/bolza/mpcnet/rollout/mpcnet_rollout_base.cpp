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

#include "bolza/mpcnet/rollout/mpcnet_rollout_base.hpp"

#include "bolza/mpcnet/control/mpcnet_behavioral_controller.hpp"

namespace bolza {
namespace mpcnet {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPCNetRolloutBase::set(scalar_t alpha, const std::string& policyFilePath, const SystemObservation& initialObservation,
                            const ModeSchedule& modeSchedule, const TargetTrajectories& targetTrajectories) {
  // init system observation
  systemObservation_ = initialObservation;

  // reset mpc
  mpcPtr_->reset();

  // prepare learned controller
  mpcnetPtr_->loadPolicyModel(policyFilePath);

  // reset rollout, i.e. reset the internal simulator state (e.g. relevant for RaiSim)
  rolloutPtr_->resetRollout();

  // update the reference manager
  referenceManagerPtr_->setModeSchedule(modeSchedule);
  referenceManagerPtr_->setTargetTrajectories(targetTrajectories);

  // set up behavioral controller with mixture parameter alpha and learned controller
  behavioralControllerPtr_->setAlpha(alpha);
  behavioralControllerPtr_->setLearnedController(*mpcnetPtr_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPCNetRolloutBase::Step(scalar_t timeStep) {
  // run mpc
  if (!mpcPtr_->run(systemObservation_.time, systemObservation_.state)) {
    throw std::runtime_error("[MPCNetRolloutBase::Step] main routine of MPC returned false.");
  }

  // update primal solution
  primalSolution_ = mpcPtr_->getSolverPtr()->primalSolution(mpcPtr_->getSolverPtr()->getFinalTime());

  // update behavioral controller with MPC controller
  behavioralControllerPtr_->setOptimalController(*primalSolution_.controllerPtr_);

  // forward simulate system with behavioral controller
  scalar_array_t timeTrajectory;
  size_array_t postEventIndicesStock;
  vector_array_t stateTrajectory;
  vector_array_t inputTrajectory;
  rolloutPtr_->run(primalSolution_.timeTrajectory_.front(), primalSolution_.stateTrajectory_.front(),
                   primalSolution_.timeTrajectory_.front() + timeStep, behavioralControllerPtr_.get(), primalSolution_.modeSchedule_,
                   timeTrajectory, postEventIndicesStock, stateTrajectory, inputTrajectory);

  // update system observation
  systemObservation_.time = timeTrajectory.back();
  systemObservation_.state = stateTrajectory.back();
  systemObservation_.input = inputTrajectory.back();
  systemObservation_.mode = primalSolution_.modeSchedule_.modeAtTime(systemObservation_.time);

  // check forward simulated system
  if (!mpcnetDefinitionPtr_->isValid(systemObservation_.time, systemObservation_.state, referenceManagerPtr_->getModeSchedule(),
                                     referenceManagerPtr_->getTargetTrajectories())) {
    throw std::runtime_error("MPCNetDataGeneration::Run Tuple (time, state, modeSchedule, targetTrajectories) is not valid.");
  }
}

}  // namespace mpcnet
}  // namespace bolza
