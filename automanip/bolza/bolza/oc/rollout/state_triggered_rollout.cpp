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

#include "bolza/oc/rollout/state_triggered_rollout.hpp"

#include <bolza/core/control/state_based_linear_controller.hpp>
#include <bolza/oc/rollout/root_finder.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateTriggeredRollout::StateTriggeredRollout(const ControlledSystemBase& systemDynamics, rollout::Settings rolloutSettings)
    : RolloutBase(std::move(rolloutSettings)),
      systemDynamicsPtr_(systemDynamics.clone()),
      systemEventHandlersPtr_(new StateTriggeredEventHandler(this->settings().timeStep)) {
  // construct dynamicsIntegratorsPtr
  dynamicsIntegratorPtr_ = std::move(NewIntegrator(this->settings().integratorType, systemEventHandlersPtr_));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t StateTriggeredRollout::Run(scalar_t initTime, const vector_t& initState, scalar_t finalTime, ControllerBase* controller,
                                    ModeSchedule& modeSchedule, scalar_array_t& timeTrajectory, size_array_t& postEventIndices,
                                    vector_array_t& stateTrajectory, vector_array_t& inputTrajectory) {
  if (initTime > finalTime) {
    throw std::runtime_error("[StateTriggeredRollout::Run] The initial time should be less-equal to the final time!");
  }
  if (controller == nullptr) {
    throw std::runtime_error("[StateTriggeredRollout::Run] Controller is not set!");
  }

  // max number of steps for integration
  const auto maxNumSteps = static_cast<size_t>(this->settings().maxNumStepsPerSecond * std::max(1.0, finalTime - initTime));

  // clearing the output trajectories
  modeSchedule.clear();
  timeTrajectory.clear();
  timeTrajectory.reserve(maxNumSteps + 1);
  stateTrajectory.clear();
  stateTrajectory.reserve(maxNumSteps + 1);
  inputTrajectory.clear();
  inputTrajectory.reserve(maxNumSteps + 1);
  postEventIndices.clear();

  // set controller
  StateBasedLinearController trajectorySpreadingController;
  if (this->settings().useTrajectorySpreadingController) {
    trajectorySpreadingController.setController(controller);
    systemDynamicsPtr_->setController(&trajectorySpreadingController);
  } else {
    systemDynamicsPtr_->setController(controller);
  }

  // reset function calls counter
  systemDynamicsPtr_->resetNumFunctionCalls();

  // reset the event class
  systemEventHandlersPtr_->reset();

  RootFinder rootFinder(this->settings().rootFindingAlgorithm);  // root-finding algorithm

  // TODO: this should be the current mode
  size_t eventID = 0;
  scalar_t t0 = initTime;
  scalar_t t1 = finalTime;
  vector_t x0 = initState;
  modeSchedule.modeSequence.push_back(eventID);

  bool refining = false;
  int k_u = 0;                    // control input iterator
  int singleEventIterations = 0;  // iterations for a single event
  int numTotalIterations = 0;     // overall number of iterations

  while (true) {  // keeps looping until end time condition is fulfilled, after which the loop is broken
    bool triggered = false;
    try {
      Observer Observer(&stateTrajectory, &timeTrajectory);  // concatenate trajectory
      dynamicsIntegratorPtr_->integrateAdaptive(*systemDynamicsPtr_, observer, x0, t0, t1, this->settings().timeStep,
                                                this->settings().absTolODE, this->settings().relTolODE, maxNumSteps);
    } catch (const size_t& e) {
      eventID = e;
      triggered = true;
    }
    // calculate guard surface value of last query state and time
    const scalar_t queryTime = timeTrajectory.back();
    const vector_t queryState = stateTrajectory.back();
    const vector_t guardSurfaces = systemDynamicsPtr_->computeGuardSurfaces(queryTime, queryState);
    const scalar_t queryGuard = guardSurfaces[eventID];

    // accuracy conditions on the obtained query guard and width of time window
    const bool guardAccuracyCondition = std::fabs(queryGuard) < this->settings().absTolODE;
    const bool timeAccuracyCondition = std::fabs(t1 - t0) < this->settings().absTolODE;
    const bool accuracyCondition = guardAccuracyCondition || timeAccuracyCondition;
    // condition to check whether max number of iterations has not been reached, to prevent an infinite loop
    const bool maxNumIterationsReached = singleEventIterations >= this->settings().maxSingleEventIterations;

    // remove the element past the guard surface if the event handler was triggered
    // (Due to checking in EventHandler this can only happen to the last element of the trajectory)
    // Exception is when the element is outside the guardSurface but within tolerance
    if (triggered && !accuracyCondition) {
      stateTrajectory.pop_back();
      timeTrajectory.pop_back();
    }
    triggered = false;

    // compute control input trajectory and concatenate to inputTrajectory
    if (this->settings().reconstructInputTrajectory) {
      for (; k_u < timeTrajectory.size(); k_u++) {
        inputTrajectory.emplace_back(systemDynamicsPtr_->controllerPtr()->computeInput(timeTrajectory[k_u], stateTrajectory[k_u]));
      }  // end of k_u loop
    }

    // end time condition to detect end of simulation, means iteration procedure is done
    if (numerics::almost_eq(finalTime, timeTrajectory.back())) {
      break;
    }

    // accuracy condition for event refinement. If sufficiently accurate crossing location has been determined
    if (accuracyCondition || maxNumIterationsReached) {
      // set new begin/end time and begin state
      t0 = queryTime + numeric_traits::WeakEpsilon<scalar_t>();
      t1 = finalTime;
      // compute jump
      x0 = systemDynamicsPtr_->computeJumpMap(queryTime, queryState);

      // append the event to array with event indices
      postEventIndices.push_back(stateTrajectory.size());

      // append to modeSchedule
      modeSchedule.eventTimes.push_back(queryTime);
      modeSchedule.modeSequence.push_back(eventID);

      // determine guard surface cross value and update the eventHandler
      vector_t guardSurfacesCross = systemDynamicsPtr_->computeGuardSurfaces(t0, x0);
      // updates the last event triggering times of Event Handler
      systemEventHandlersPtr_->setLastEvent(t0, guardSurfacesCross);

      // reset relevant boolean and counter
      refining = false;
      singleEventIterations = 0;
    } else {           // otherwise keep or start refining
      if (refining) {  // apply the rules of the root-finding method to continue refining
        rootFinder.updateBracket(queryTime, queryGuard);
      } else {  // properly configure root-finding method to start refining
        const scalar_t& timeBefore = timeTrajectory.back();
        const vector_t& stateBefore = stateTrajectory.back();
        const vector_t guardSurfacesBefore = systemDynamicsPtr_->computeGuardSurfaces(timeBefore, stateBefore);
        const scalar_t& guardBefore = guardSurfacesBefore[eventID];

        rootFinder.setInitBracket(timeBefore, queryTime, guardBefore, queryGuard);
        refining = true;
      }
      t1 = rootFinder.getNewQuery();
      t0 = timeTrajectory.back();
      x0 = stateTrajectory.back();

      stateTrajectory.pop_back();
      timeTrajectory.pop_back();
      inputTrajectory.pop_back();
      k_u--;
    }
    singleEventIterations++;
    numTotalIterations++;
  }  // end of while loop

  // check for the numerical stability
  this->checkNumericalStability(*controller, timeTrajectory, postEventIndices, stateTrajectory, inputTrajectory);

  return stateTrajectory.back();
}

}  // namespace bolza
