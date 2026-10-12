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

#include <exception>

#include <bolza/core/integration/system_event_handler.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<bool, size_t> SystemEventHandler::CheckEvent(OdeBase& system, scalar_t time, const vector_t& state) {
  return {false, 0};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SystemEventHandler::HandleEvent(OdeBase& system, scalar_t time, const vector_t& state) {
  if (killIntegration_) {
    throw std::runtime_error("Integration terminated due to an external signal triggered by a program.");
  }

  // derived class events
  size_t eventID;
  bool event;
  std::tie(event, eventID) = this->checkEvent(system, time, state);
  if (event) {
    throw eventID;
  }
}

}  // namespace bolza
