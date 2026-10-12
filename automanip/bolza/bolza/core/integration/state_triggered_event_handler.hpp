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

#pragma once

/**
 * @file state_triggered_event_handler.hpp
 * @brief State triggered event handler class for ode solvers.
 */


#include <bolza/core/integration/system_event_handler.hpp>

namespace bolza {

/**
 * State triggered event handler class for ode solvers.
 */
class StateTriggeredEventHandler final : public SystemEventHandler {
 public:
  /**
   * Constructor
   *
   * @param [in] minEventTimeDifference: Minimum accepted time difference between two consecutive events.
   */
  explicit StateTriggeredEventHandler(scalar_t minEventTimeDifference);

  /**
   * Default destructor
   */
  ~StateTriggeredEventHandler() override = default;

  /**
   * Checks whether an event is activated. If true, the method should also return
   * a "Non-Negative" ID which indicates the a unique ID for the active events.
   *
   * @param [in] system: System dynamics
   * @param [in] time: The current time.
   * @param [in] state: The current state vector.
   * @return pair of event flag and eventID
   */
  std::pair<bool, size_t> CheckEvent(OdeBase& system, scalar_t time, const vector_t& state) override;

  /**
   * Sets parameters to control event times detection.
   *
   * @param [in] lastEventTriggeredTime: Last Time that an event is triggered.
   * @param [in] lastGuardSurfacesValues: The value of the guard functions at lastEventTriggeredTime.
   */
  void SetLastEvent(scalar_t lastEventTriggeredTime, const vector_t& lastGuardSurfacesValues);

  /**
   * Gets the value of the guard surfaces.
   *
   * @return The value of the guard surfaces.
   */
  const vector_t& GetGuardSurfacesValues() const;

  /**
   * Gets the minimum acceptable time in between two consecutive events.
   *
   * @return The value of minEventTimeDifference_
   */
  scalar_t GetminEventTimeDifference() const;

  /**
   * Resets the class.
   */
  void Reset() override;

 protected:
  scalar_t minEventTimeDifference_;
  vector_t guardSurfacesValuesCurrent_;
  vector_t guardSurfacesValuesPrevious_;
  /**
   * @brief Lowest.
   * @return Result value.
   */
  scalar_t lastEventTriggeredTime_ = std::numeric_limits<scalar_t>::lowest();
};

}  // namespace bolza
