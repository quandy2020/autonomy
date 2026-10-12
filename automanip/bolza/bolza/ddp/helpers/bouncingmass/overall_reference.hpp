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

#include <bolza/core/types.hpp>

#include "helpers/bouncingmass/reference.hpp"

using scalar_t = bolza::scalar_t;
using vector_t = bolza::vector_t;
using matrix_t = bolza::matrix_t;
using scalar_array_t = bolza::scalar_array_t;
using vector_array_t = bolza::vector_array_t;

/*
 * 	This class constructs, contains and provides the reference for the
 * 	BouncingMass SLQ test. The reference is based on the reference presented
 * 	in [On Optimal Trajectory Tracking for Mechanical Systems with Unilateral Constraints
 * by M. Rijnen]
 */
class OverallReference {
 public:
  enum { STATE_DIM = 3, INPUT_DIM = 1 };
  /*
   * Constructor
   */
  OverallReference() = default;

  /*
   * Constructor
   *
   * @param [in] trajTimes: list of times at which the reference is defined
   * @param [in] trajState: list of waypoints at which the reference is defined
   */
  OverallReference(const scalar_array_t& trajTimes, const vector_array_t& trajState);

  /*
   * Calculate the input at a certain time
   *
   * @param [in] time: time moment at which the input is calculated
   * @return input corresponding to time
   */
  vector_t GetInput(scalar_t time) const;

  /*
   * Calculate the reference state at a certain time
   *
   * @param [in] time: time moment at which the input is calculated
   * @return state corresponding to time
   */
  vector_t GetState(scalar_t time) const;

  /*
   * Calculate the reference state at a certain time and mode
   *
   * @param [in] idx: mode at which the input is calculated
   * @param [in] time: time moment at which the input is calculated
   * @return state corresponding to time and mode
   */
  vector_t GetState(int idx, scalar_t time) const;

  /*
   * Extend the reference past the event times, by integrating the input signal
   * without applying the jump map
   *
   * @param [in] delta: length of the extensions
   */
  void Extendref(scalar_t delta);

  /*
   * Display the reference
   *
   * @param [in] i: choice of which section of the reference is displayed
   */
  void Display(int i);

 private:
  /*
   * Jump map of the system
   * @param [in] x: State before event
   * @return state after event
   */
  vector_t JumpMap(const vector_t& x) const;

  std::vector<Reference> References_;
  std::vector<scalar_t> switchtimes_;
};
