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

#include <Eigen/Dense>

#include <bolza/core/types.hpp>

using scalar_t = bolza::scalar_t;
using vector_t = bolza::vector_t;
using matrix_t = bolza::matrix_t;
using scalar_array_t = bolza::scalar_array_t;
using vector_array_t = bolza::vector_array_t;

/*
 * 	This class constructs, contains a section of the reference, which is contained in
 * 	OverallReference and used in BouncingMass SLQ test.
 * 	The reference is based on the reference presented in
 * 	"On Optimal Trajectory Tracking for Mechanical Systems with Unilateral Constraints" by M. Rijnen
 */
class Reference {
 public:
  /*
   * Constructor
   *
   * @param [in] t0: Time at which the first waypoint of the reference is defined
   * @param [in] t1: Time at which the second waypoint of the reference is defined
   * @param [in] p0: First waypoint of the reference [position,velocity,acceleration]
   * @param [in] p1: Second waypoint of the reference
   */
  Reference(scalar_t t0, scalar_t t1, const vector_t& p0, const vector_t& p1);

  /*
   * Obtain reference input at the current time
   *
   * @param [in] time: current time
   * @return current reference input
   */
  vector_t GetInput(const scalar_t time) const;

  /*
   * Obtain reference state at current time
   *
   * @param [in] time: current time
   * @return current reference state
   */
  vector_t GetState(const scalar_t time) const;

  /*
   * Extend the reference by integrating the input signal of the next
   * and previous sections
   *
   * @param [in] delta: extension time
   * @param [in] refPre: pointer to previous section of the reference
   * @param [in] refPost: pointer to the next section of the reference
   */
  void Extendref(scalar_t delta, Reference* refPre, Reference* refPost);

  /* Display the reference */
  void Display();

  scalar_t t0_;
  scalar_t t1_;

 private:
  /*
   * 	Find the polynomial coefficients corresponding to the posed constraints
   * 	on position, velocity and accleration at the posed times
   *
   * 	@param [in]	t0:	Time moment of first waypoint
   * 	@param [in] t1: time moment of second waypoint
   * 	@param [in] p0: first waypoint
   * 	@param [in] p1: second waypoint
   */
  Eigen::Matrix<scalar_t, 6, 1> Create5thOrdPol(scalar_t t0, scalar_t t1, const vector_t& p0, const vector_t& p1) const;

  /*
   * 	Find the derivative of the polynomial described by a
   * 	set of coefficients
   *
   * 	@param [in]	pol: coefficients describing the fifth order polynomial
   */
  Eigen::Matrix<scalar_t, 6, 1> Polyder(Eigen::Matrix<scalar_t, 6, 1> pol);

  /*
   * 	Interpolate the extended reference signals to find the reference
   * 	corresponding to the current time
   *
   * 	@param [in]	time:	current time
   * 	@param [out] x: current state
   */
  void interpolate_ext(scalar_t time, vector_t& x) const;

  Eigen::Matrix<scalar_t, 6, 1> polU_;
  Eigen::Matrix<scalar_t, 6, 1> polX_;
  Eigen::Matrix<scalar_t, 6, 1> polV_;

  vector_array_t xPre_;
  scalar_array_t tPre_;
  vector_array_t xPost_;
  scalar_array_t tPost_;
};
