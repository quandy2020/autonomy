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
 * @file system_event_handler.hpp
 * @brief System event ID. all values are negative.
 */


#include <atomic>
#include <limits>
#include <memory>
#include <utility>

#include <bolza/core/types.hpp>
#include <bolza/core/integration/ode_base.hpp>

namespace bolza {

/**
 * System event ID. all values are negative.
 */
enum SysEventId {
  killIntegration = -1,  //!< killIntegration: kill integration due to an external signal.
  maxCall = -2           //!< maximum number of function calls.
};

/**
 * Event handler class for ode solvers.
 */
class SystemEventHandler {
 public:
  /** Default constructor */
  SystemEventHandler() = default;

  /** Default destructor */
  virtual ~SystemEventHandler() = default;

  /**
   * Checks whether an event is activated. If true, the method should also return
   * a "Non-Negative" ID which indicates the a unique ID for the active events.
   *
   * @param [in] system: System dynamics
   * @param [in] time: The current time.
   * @param [in] state: The current state vector.
   * @return pair of event flag and eventID
   */
  virtual std::pair<bool, size_t> CheckEvent(OdeBase& system, scalar_t time, const vector_t& state);

  /**
   * The operation should be performed if an event is activated.
   *
   * @param [in] system: System dynamics
   * @param [in] time: The current time.
   * @param [in] state: The current state vector.
   */
  void HandleEvent(OdeBase& system, scalar_t time, const vector_t& state);

  /** Resets the class. */
  virtual void Reset() { killIntegration_ = false; };

 private:
  /** Copy constructor */
  SystemEventHandler(const SystemEventHandler& rhs) : SystemEventHandler() {}

 public:
  std::atomic_bool killIntegration_ = {false};
};

}  // namespace bolza
