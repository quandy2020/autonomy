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

#include <bolza/core/integration/state_triggered_event_handler.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateTriggeredEventHandler::StateTriggeredEventHandler(scalar_t minEventTimeDifference)
    : SystemEventHandler(), minEventTimeDifference_(minEventTimeDifference) {
  Reset();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<bool, size_t> StateTriggeredEventHandler::CheckEvent(OdeBase& system, scalar_t time, const vector_t& state) {
  // state-triggered event
  guardSurfacesValuesCurrent_ = system.ComputeGuardSurfaces(time, state);

  size_t eventID = 0;
  bool eventTriggered = false;
  if (time - lastEventTriggeredTime_ > minEventTimeDifference_) {
    for (size_t i = 0; i < guardSurfacesValuesPrevious_.size(); i++) {
      if (guardSurfacesValuesCurrent_(i) <= 0 && guardSurfacesValuesPrevious_(i) > 0) {
        eventID = i;
        eventTriggered = true;
      }
    }
  }

  // update guard surfaces if event is not triggered
  if (!eventTriggered) {
    guardSurfacesValuesPrevious_ = guardSurfacesValuesCurrent_;
  }

  return {eventTriggered, eventID};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void StateTriggeredEventHandler::SetLastEvent(scalar_t lastEventTriggeredTime, const vector_t& lastGuardSurfacesValues) {
  lastEventTriggeredTime_ = lastEventTriggeredTime;
  guardSurfacesValuesPrevious_ = lastGuardSurfacesValues;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
const vector_t& StateTriggeredEventHandler::GetGuardSurfacesValues() const {
  return guardSurfacesValuesPrevious_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t StateTriggeredEventHandler::GetminEventTimeDifference() const {
  return minEventTimeDifference_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void StateTriggeredEventHandler::Reset() {
  SystemEventHandler::Reset();
  lastEventTriggeredTime_ = std::numeric_limits<scalar_t>::lowest();
  guardSurfacesValuesPrevious_.setZero(0);
}

}  // namespace bolza
