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
 * @file mpc_settings.hpp
 * @brief This structure holds all setting parameters for the MPC class.
 */


#include <iostream>
#include <string>

#include <bolza/core/types.hpp>

namespace bolza {
namespace mpc {

/**
 * This structure holds all setting parameters for the MPC class.
 */
struct Settings {
  /** MPC time horizon length in seconds. */
  scalar_t timeHorizon_ = 1.0;
  /**
   * The time window (in seconds) for retrieving the optimized outputs (controller and state-input
   * trajectories). Any negative number will be interpreted as the whole time horizon.
   * */
  scalar_t solutionTimeWindow_ = -1;

  /** This value determines to display the log output of MPC. */
  bool debugPrint_ = false;

  /** This value determines to initialize the SLQ with the controller from previous call (warm start)
   * or the given operating trajectories (cold start). */
  bool coldStart_ = false;

  /**
   * MPC loop frequency in Hz. This setting is only used in Dummy_Loop for testing. If set to a
   * positive number, THe MPC loop will be simulated to run by the given frequency (note that this
   * might not be the MPC's real-time frequency). Any negative number will cause the MPC loop to run
   * by its maximum possible frequency.
   */
  scalar_t mpcDesiredFrequency_ = -1;
  /**
   * MRT loop frequency in Hz. This setting is only used in Dummy_Loop for testing. This should always
   * set to a positive number which can be interpreted as the tracking controller's frequency.
   */
  scalar_t mrtDesiredFrequency_ = 100.0;
};

/**
 * Loads the MPC settings from a given file.
 *
 * @param [in] filename: File name which contains the configuration data.
 * @param [in] fieldName: Field name which contains the configuration data.
 * @param [in] verbose: Flag to determine whether to print out the loaded settings or not.
 * @return The MPC settings
 */
Settings LoadSettings(const std::string& filename, const std::string& fieldName = "mpc", bool verbose = true);

}  // namespace mpc
}  // namespace bolza
