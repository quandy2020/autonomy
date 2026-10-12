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
 * @file root_finder.hpp
 * @brief This class is used in StateTriggerdRollout to find a new query point,.
 */


#include <iostream>
#include <utility>
#include <vector>

#include <bolza/core/types.hpp>

#include "root_finder_type.hpp"

namespace bolza {

/**
 * This class is used in StateTriggerdRollout to find a new query point,
 * based on a previously known interval, which is the best guess to find
 * the root of the guardSurface.
 *
 * It assumes that on initialization an interval is known on which the
 * guard function changes sign, and that within this interval a single root
 * is located.
 *
 * The implemented algorithms are:
 * Anderson & Bjorck Method (default):
 * from: [A New High Order Method of Regula Falsi Type for Computing a Root of an Equation
 *        by N.Anderson and A. Bjorck]
 * Pegasus method:
 * from: [The "Pegasus" Method for Computing the Root of an Equation
 *        by M. Dowell and P. Jarrat]
 * Illinois method:
 * from: [A Modified Regula Falsi Method for Computing the Root of an Equation
 *        by M. Dowell and P. Jarrat]
 * Regular Regula Falsi:
 * form: [Improved Algorithms of Illinois-Type for the Numerical Solution of Non-
 * 	      Linear Equations by J.A. Ford]
 */
class RootFinder {
 public:
  using pair_t = std::pair<scalar_t, scalar_t>;

  /**
   * Constructor.
   *
   * @param [in] rootFindingAlgorithm: The root-finding algorithm. The default algorithm is "Anderson & Bjorck" method.
   */
  explicit RootFinder(RootFinderType rootFindingAlgorithm = RootFinderType::ANDERSON_BJORCK)
      /**
       * @brief Root Finding Algorithm.
       * @param [in] rootFindingAlgorithm: Root Finding Algorithm.
       * @return Result value.
       */
      : rootFindingAlgorithm_(rootFindingAlgorithm) {}

  /**
   * Default destructor.
   */
  ~RootFinder() = default;

  /**
   * Sets the initial bracket when the RootFinding method is initialized.
   * Normally done when first zero crossing is detected
   *
   * @param [in] timeInt: Pair of two time moments.
   * @param [in] guardInt: Pair of two function values at time_int times, should have opposite sign.
   */
  void SetInitBracket(pair_t timeInt, pair_t guardInt);

  /**
   * Sets the initial bracket when the RootFinding method is initialized.
   * Normally done when first zero crossing is detected.
   *
   * @param [in] t0: First time of bracketing interval.
   * @param [in] t1: Second time of bracketing interval.
   * @param [in] f0: Function value corresponding to t0.
   * @param [in] f1: Function value corresponding to t1, of opposite sign to f0.
   */
  void SetInitBracket(scalar_t t0, scalar_t t1, scalar_t f0, scalar_t f1) {
    /**
     * @brief Constructs a SetInitBracket.
     */
    SetInitBracket(std::make_pair(t0, t1), std::make_pair(f0, f1));
  }

  /**
   * Updates the current bracket, based on the sign of the new query point.
   *
   * @param [in] query: Time moment of last query point
   * @param [in] fQuery: Function evaluation of last query time
   */
  void UpdateBracket(const scalar_t& query, const scalar_t& fQuery);

  /**
   * Uses (adapted-) regula falsi method to obtain a new query point
   *
   * @return Time moment of new query point
   */
  scalar_t GetNewQuery() {
    const scalar_t& fa = guardInt_.first;
    const scalar_t& ta = timeInt_.first;
    const scalar_t& fb = guardInt_.second;
    const scalar_t& tb = timeInt_.second;
    return (ta * fb - tb * fa) / (fb - fa);
  }

  /**
   * Displays relevant bracketing information
   */
  void Display();

 private:
  RootFinderType rootFindingAlgorithm_;
  pair_t timeInt_;
  pair_t guardInt_;
};

}  // namespace bolza
