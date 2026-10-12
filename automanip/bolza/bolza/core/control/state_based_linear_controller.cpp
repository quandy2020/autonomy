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

#include <bolza/core/control/state_based_linear_controller.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void StateBasedLinearController::SetController(ControllerBase* ctrlPtr) {
  if (ctrlPtr == nullptr) {
    throw std::runtime_error("The controller pointer is null!");
  }
  ctrlPtr_ = ctrlPtr;
  ctrlEventTimes_ = ctrlPtr->ControllerEventTimes();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t StateBasedLinearController::ComputeTrajectorySpreadingInput(scalar_t t, const vector_t& x, const scalar_array_t& ctrlEventTimes,
                                                                     ControllerBase* ctrlPtr) {
  size_t currentMode = static_cast<size_t>(x.tail(1).value());
  size_t numEvents = ctrlEventTimes.size();

  if (numEvents == 0)  // Simple case in which the controller does not contain any events
  {
    return ctrlPtr->ComputeInput(t, x);
  }

  scalar_t tauMinus = (numEvents > currentMode) ? ctrlEventTimes[currentMode] : ctrlEventTimes.back();
  scalar_t tau = (numEvents > currentMode + 1) ? ctrlEventTimes[currentMode + 1] : ctrlEventTimes.back();
  scalar_t tauPlus = (numEvents > currentMode + 2) ? ctrlEventTimes[currentMode + 2] : ctrlEventTimes.back();

  bool pastAllEvents = (currentMode >= numEvents - 1) && (t > tauMinus);
  const scalar_t eps = numeric_traits::WeakEpsilon<scalar_t>();

  if (pastAllEvents) {
    return ctrlPtr->ComputeInput(t, x);
    // return normal input signal
  } else if (t < tauMinus) {
    // if event happened before the event time for which the controller was designed
    return ctrlPtr->ComputeInput(tauMinus + 2.0 * eps, x);
    // request input 1 epsilon after the designed event time
  } else if (t > tau) {
    // if event has not happened yet at the event time for which the controller was designed
    return ctrlPtr->ComputeInput(tau - eps, x);
    // request input 1 epsilon before the designed event time
  }
  // normal case: t > tauMinus && t < tau
  return ctrlPtr->ComputeInput(t, x);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t StateBasedLinearController::ComputeInput(scalar_t t, const vector_t& x) {
  return ComputeTrajectorySpreadingInput(t, x, ctrlEventTimes_, ctrlPtr_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void StateBasedLinearController::Concatenate(const ControllerBase* nextController, int index, int length) {
  ctrlPtr_->Concatenate(nextController, index, length);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
int StateBasedLinearController::size() const {
  return ctrlPtr_->size();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ControllerType StateBasedLinearController::GetType() const {
  return ctrlPtr_->GetType();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void StateBasedLinearController::clear() {
  ctrlPtr_->clear();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool StateBasedLinearController::empty() const {
  return ctrlPtr_->empty();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void StateBasedLinearController::Display() const {
  ctrlPtr_->Display();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateBasedLinearController* StateBasedLinearController::Clone() const {
  return new StateBasedLinearController(*this);
}

}  // namespace bolza
