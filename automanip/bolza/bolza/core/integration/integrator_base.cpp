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

#include <bolza/core/integration/integrator_base.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
IntegratorBase::IntegratorBase(std::shared_ptr<SystemEventHandler> eventHandlerPtr /*= nullptr*/)
    : eventHandlerPtr_(std::move(eventHandlerPtr)) {
  if (eventHandlerPtr_ == nullptr) {
    eventHandlerPtr_ = std::make_shared<SystemEventHandler>();
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
IntegratorBase::system_func_t IntegratorBase::SystemFunction(OdeBase& system, int maxNumSteps) const {
  return [&system, maxNumSteps](const vector_t& x, vector_t& dxdt, scalar_t t) {
    dxdt = system.computeFlowMap(t, x);
    // max number of function calls
    if (system.incrementNumFunctionCalls() > maxNumSteps) {
      std::stringstream msg;
      msg << "Integration terminated since the maximum number of function calls is reached. State at termination time " << t << ":\n["
          << x.transpose() << "]\n";
      throw std::runtime_error(msg.str());
    }
  };
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void IntegratorBase::IntegrateConst(OdeBase& system, Observer& observer, const vector_t& initialState, scalar_t startTime,
                                    scalar_t finalTime, scalar_t dt, int maxNumSteps /*= std::numeric_limits<int>::max()*/) {
  observer_func_t callback = [&](const vector_t& x, scalar_t t) {
    observer.observe(x, t);
    eventHandlerPtr_->handleEvent(system, t, x);
  };
  RunIntegrateConst(SystemFunction(system, maxNumSteps), callback, initialState, startTime, finalTime, dt);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void IntegratorBase::IntegrateAdaptive(OdeBase& system, Observer& observer, const vector_t& initialState, scalar_t startTime,
                                       scalar_t finalTime, scalar_t dtInitial /*= 0.01*/, scalar_t AbsTol /*= 1e-6*/,
                                       scalar_t RelTol /*= 1e-3*/, int maxNumSteps /*= std::numeric_limits<int>::max()*/) {
  observer_func_t callback = [&](const vector_t& x, scalar_t t) {
    observer.observe(x, t);
    eventHandlerPtr_->handleEvent(system, t, x);
  };
  RunIntegrateAdaptive(SystemFunction(system, maxNumSteps), callback, initialState, startTime, finalTime, dtInitial, AbsTol, RelTol);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void IntegratorBase::IntegrateTimes(OdeBase& system, Observer& observer, const vector_t& initialState,
                                    typename scalar_array_t::const_iterator beginTimeItr,
                                    typename scalar_array_t::const_iterator endTimeItr, scalar_t dtInitial /*= 0.01*/,
                                    scalar_t AbsTol /*= 1e-6*/, scalar_t RelTol /*= 1e-3*/,
                                    int maxNumSteps /*= std::numeric_limits<int>::max()*/) {
  observer_func_t callback = [&](const vector_t& x, scalar_t t) {
    observer.observe(x, t);
    eventHandlerPtr_->handleEvent(system, t, x);
  };
  RunIntegrateTimes(SystemFunction(system, maxNumSteps), callback, initialState, beginTimeItr, endTimeItr, dtInitial, AbsTol, RelTol);
}

}  // namespace bolza
