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

#include "bolza/mpc/mrt_base.hpp"

#include <bolza/oc/rollout/time_triggered_rollout.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MRTBase::MRTBase() {
  Reset();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRTBase::Reset() {
  std::lock_guard<std::mutex> lock(bufferMutex_);

  policyReceivedEver_ = false;
  newPolicyInBuffer_ = false;
  mrtTrylockWarningCount_ = 0;

  activeCommandPtr_.reset();
  bufferCommandPtr_.reset();
  activePrimalSolutionPtr_.reset();
  bufferPrimalSolutionPtr_.reset();
  activePerformanceIndicesPtr_.reset();
  bufferPerformanceIndicesPtr_.reset();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
const CommandData& MRTBase::GetCommand() const {
  if (activeCommandPtr_ != nullptr) {
    return *activeCommandPtr_;
  } else {
    throw std::runtime_error("[MRTBase::GetCommand] UpdatePolicy() should be called first!");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
const PrimalSolution& MRTBase::GetPolicy() const {
  if (activePrimalSolutionPtr_ != nullptr) {
    return *activePrimalSolutionPtr_;
  } else {
    throw std::runtime_error("[MRTBase::GetPolicy] UpdatePolicy() should be called first!");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
const PerformanceIndex& MRTBase::GetPerformanceIndices() const {
  if (activePerformanceIndicesPtr_ != nullptr) {
    return *activePerformanceIndicesPtr_;
  } else {
    throw std::runtime_error("[MRTBase::GetPerformanceIndices] UpdatePolicy() should be called first!");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRTBase::InitRollout(const RolloutBase* rolloutPtr) {
  rolloutPtr_.reset(rolloutPtr->Clone());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRTBase::EvaluatePolicy(scalar_t currentTime, const vector_t& currentState, vector_t& mpcState, vector_t& mpcInput, size_t& mode) {
  if (activePrimalSolutionPtr_ == nullptr) {
    throw std::runtime_error("[MRTBase::EvaluatePolicy] UpdatePolicy() should be called first!");
  }

  if (currentTime > activePrimalSolutionPtr_->timeTrajectory_.back()) {
    std::cerr << "The requested currentTime is greater than the received plan: " << std::to_string(currentTime) << ">"
              << std::to_string(activePrimalSolutionPtr_->timeTrajectory_.back()) << "\n";
  }

  mpcInput = activePrimalSolutionPtr_->controllerPtr_->ComputeInput(currentTime, currentState);
  mpcState =
      LinearInterpolation::Interpolate(currentTime, activePrimalSolutionPtr_->timeTrajectory_, activePrimalSolutionPtr_->stateTrajectory_);

  mode = activePrimalSolutionPtr_->modeSchedule_.ModeAtTime(currentTime);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRTBase::RolloutPolicy(scalar_t currentTime, const vector_t& currentState, const scalar_t& timeStep, vector_t& mpcState,
                             vector_t& mpcInput, size_t& mode) {
  if (rolloutPtr_ == nullptr) {
    throw std::runtime_error("[MRTBase::RolloutPolicy] rollout class is not set! Use InitRollout() to initialize it!");
  }

  if (activePrimalSolutionPtr_ == nullptr) {
    throw std::runtime_error("[MRTBase::RolloutPolicy] UpdatePolicy() should be called first!");
  }

  if (currentTime > activePrimalSolutionPtr_->timeTrajectory_.back()) {
    std::cerr << "The requested currentTime is greater than the received plan: " << std::to_string(currentTime) << ">"
              << std::to_string(activePrimalSolutionPtr_->timeTrajectory_.back()) << "\n";
  }

  // perform a rollout
  scalar_array_t timeTrajectory;
  size_array_t postEventIndicesStock;
  vector_array_t stateTrajectory, inputTrajectory;
  const scalar_t finalTime = currentTime + timeStep;
  rolloutPtr_->Run(currentTime, currentState, finalTime, activePrimalSolutionPtr_->controllerPtr_.get(),
                   activePrimalSolutionPtr_->modeSchedule_, timeTrajectory, postEventIndicesStock, stateTrajectory, inputTrajectory);

  mpcState = stateTrajectory.back();
  mpcInput = inputTrajectory.back();

  mode = activePrimalSolutionPtr_->modeSchedule_.ModeAtTime(finalTime);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool MRTBase::UpdatePolicy() {
  std::unique_lock<std::mutex> lock(bufferMutex_, std::try_to_lock);
  if (lock.owns_lock()) {
    mrtTrylockWarningCount_ = 0;
    if (newPolicyInBuffer_) {
      // update the active solution from buffer
      activeCommandPtr_.swap(bufferCommandPtr_);
      activePrimalSolutionPtr_.swap(bufferPrimalSolutionPtr_);
      activePerformanceIndicesPtr_.swap(bufferPerformanceIndicesPtr_);
      newPolicyInBuffer_ = false;  // make sure we don't swap in the old policy again

      ModifyActiveSolution(*activeCommandPtr_, *activePrimalSolutionPtr_);
      return true;
    } else {
      return false;  // No policy update: the buffer contains nothing new.
    }
  } else {
    ++mrtTrylockWarningCount_;
    if (mrtTrylockWarningCount_ > mrtTrylockWarningThreshold_) {
      std::cerr << "[MRTBase::UpdatePolicy] failed to lock the policyBufferMutex for " << mrtTrylockWarningCount_
                << " consecutive times.\n";
    }
    return false;  // No policy update: the lock could not be acquired.
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRTBase::MoveToBuffer(std::unique_ptr<CommandData> commandDataPtr, std::unique_ptr<PrimalSolution> primalSolutionPtr,
                            std::unique_ptr<PerformanceIndex> performanceIndicesPtr) {
  if (commandDataPtr == nullptr) {
    throw std::runtime_error("[MRTBase::MoveToBuffer] commandDataPtr cannot be a null pointer!");
  }

  if (primalSolutionPtr == nullptr) {
    throw std::runtime_error("[MRTBase::MoveToBuffer] primalSolutionPtr cannot be a null pointer!");
  }

  if (performanceIndicesPtr == nullptr) {
    throw std::runtime_error("[MRTBase::MoveToBuffer] performanceIndicesPtr cannot be a null pointer!");
  }

  std::lock_guard<std::mutex> lk(bufferMutex_);
  // use swap such that the old objects are destroyed after releasing the lock.
  bufferCommandPtr_.swap(commandDataPtr);
  bufferPrimalSolutionPtr_.swap(primalSolutionPtr);
  bufferPerformanceIndicesPtr_.swap(performanceIndicesPtr);

  // allow user to modify the buffer
  ModifyBufferedSolution(*bufferCommandPtr_, *bufferPrimalSolutionPtr_);

  newPolicyInBuffer_ = true;
  policyReceivedEver_ = true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRTBase::ModifyActiveSolution(const CommandData& command, PrimalSolution& primalSolution) {
  for (auto& mrtObserver : observerPtrArray_) {
    if (mrtObserver != nullptr) {
      mrtObserver->ModifyActiveSolution(command, primalSolution);
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRTBase::ModifyBufferedSolution(const CommandData& commandBuffer, PrimalSolution& primalSolutionBuffer) {
  for (auto& mrtObserver : observerPtrArray_) {
    if (mrtObserver != nullptr) {
      mrtObserver->ModifyBufferedSolution(commandBuffer, primalSolutionBuffer);
    }
  }
}

}  // namespace bolza
