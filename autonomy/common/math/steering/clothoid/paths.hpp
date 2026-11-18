/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file paths.hpp
 * @brief Clothoid Dubins / Reeds–Shepp path records and control builders.
 */

#pragma once

#include <numeric>
#include <vector>

#include "autonomy/common/math/steering/clothoid/configuration.hpp"
#include "autonomy/common/math/steering/clothoid/circle.hpp"
#include "autonomy/common/math/steering/types.hpp"

/**
 * @namespace autonomy::common::math::steering
 * @brief Clothoid and Dubins / Reeds–Shepp steering.
 */
namespace autonomy {
namespace common {
namespace math {
namespace steering {


class Path
{
public:
  /**
   * @brief Constructor.
   * @param[in] _start Start configuration.
   * @param[in] _end End configuration.
   * @param[in] _kappa Maximum curvature.
   * @param[in] _sigma Maximum sharpness.
   * @param[in] _length Path length.
   */
  Path(const Configuration &_start, const Configuration &_end, double _kappa, double _sigma, double _length);

  /**
   * @brief Start and end configuration.
   */
  Configuration start, end;

  /**
   * @brief Max. curvature (unsigned), max. sharpness (unsigned).
   */
  double kappa, sigma;

  /**
   * @brief Path length.
   */
  double length;
};

/**
 * @brief cc-dubins path types: E (Empty), S (Straight), T (Turn).
 */
namespace cc_dubins
{
enum path_type
{
  E,
  S,
  T,
  TT,
  // Dubins families:
  TST,
  TTT,
  // #####################
  TTTT
};
}
const int kNbCcDubinsPaths = 7;

class CcDubinsPath : public Path
{
public:
  /**
   * @brief Constructor.
   * @param[in] _start Start configuration.
   * @param[in] _end End configuration.
   * @param[in] _type Path family identifier.
   * @param[in] _kappa Maximum curvature.
   * @param[in] _sigma Maximum sharpness.
   * @param[in] _qi1 First intermediate configuration.
   * @param[in] _qi2 Second intermediate configuration.
   * @param[in] _qi3 Third intermediate configuration.
   * @param[in] _qi4 Fourth intermediate configuration.
   * @param[in] _cstart Start circle.
   * @param[in] _cend End circle.
   * @param[in] _ci1 First intermediate circle.
   * @param[in] _ci2 Second intermediate circle.
   * @param[in] _length Path length.
   */
  CcDubinsPath(const Configuration &_start, const Configuration &_end, cc_dubins::path_type _type, double _kappa,
                 double _sigma, Configuration *_qi1, Configuration *_qi2, Configuration *_qi3, Configuration *_qi4,
                 HcCcCircle *_cstart, HcCcCircle *_cend, HcCcCircle *_ci1, HcCcCircle *_ci2, double _length);

  /**
   * @brief Destructor.
   */
  ~CcDubinsPath();

  /**
   * @brief Alphanumeric display.
   * @param[in] eol True to print a trailing newline.
   */
  void Print(bool eol) const;

  /**
   * @brief Path type.
   */
  cc_dubins::path_type type;

  /**
   * @brief Intermediate configurations.
   */
  Configuration *qi1, *qi2, *qi3, *qi4;

  /**
   * @brief Start, end and intermediate circles.
   */
  HcCcCircle *cstart, *cend, *ci1, *ci2;
};

/**
 * @brief hc-/cc-reeds-shepp path types: E (Empty), S (Straight), T (Turn), c (Cusp).
 */
namespace hc_cc_rs
{
enum path_type
{
  E,
  S,
  T,
  TT,
  TcT,
  // Reeds-Shepp families:
  TcTcT,
  TcTT,
  TTcT,
  TST,
  TSTcT,
  TcTST,
  TcTSTcT,
  TTcTT,
  TcTTcT,
  // #####################
  TTT,
  TcST,
  TScT,
  TcScT
};
}
const int kNbHcCcRsPaths = 18;

class HcCcRsPath : public Path
{
public:
  /**
   * @brief Constructor.
   * @param[in] _start Start configuration.
   * @param[in] _end End configuration.
   * @param[in] _type Path family identifier.
   * @param[in] _kappa Maximum curvature.
   * @param[in] _sigma Maximum sharpness.
   * @param[in] _qi1 First intermediate configuration.
   * @param[in] _qi2 Second intermediate configuration.
   * @param[in] _qi3 Third intermediate configuration.
   * @param[in] _qi4 Fourth intermediate configuration.
   * @param[in] _cstart Start circle.
   * @param[in] _cend End circle.
   * @param[in] _ci1 First intermediate circle.
   * @param[in] _ci2 Second intermediate circle.
   * @param[in] _length Path length.
   */
  HcCcRsPath(const Configuration &_start, const Configuration &_end, hc_cc_rs::path_type _type, double _kappa,
                double _sigma, Configuration *_qi1, Configuration *_qi2, Configuration *_qi3, Configuration *_qi4,
                HcCcCircle *_cstart, HcCcCircle *_cend, HcCcCircle *_ci1, HcCcCircle *_ci2, double _length);

  /**
   * @brief Destructor.
   */
  ~HcCcRsPath();

  /**
   * @brief Alphanumeric display.
   * @param[in] eol True to print a trailing newline.
   */
  void Print(bool eol) const;

  /**
   * @brief Path type.
   */
  hc_cc_rs::path_type type;

  /**
   * @brief Intermediate configurations.
   */
  Configuration *qi1, *qi2, *qi3, *qi4;

  /**
   * @brief Start, end and intermediate circles.
   */
  HcCcCircle *cstart, *cend, *ci1, *ci2;
};

/**
 * @brief Checks whether two states are equal.
 * @param[in] state1 Start state.
 * @param[in] state2 Goal state.
 * @return True when the predicate holds.
 */
bool StateEqual(const State &state1, const State &state2);

/**
 * @brief Reverses a control.
 * @param[out] control Signed length, curvature, and sharpness.
 */
void ReverseControl(Control &control);

/**
 * @brief Subtracts control2 from control1.
 * @param[in] control1 Minuend control.
 * @param[in] control2 Subtrahend control.
 * @return Resulting control.
 */
Control SubtractControl(const Control &control1, const Control &control2);

/**
 * @brief Appends controls with 0 input.
 * @param[out] controls Sequence of path-segment controls.
 */
void EmptyControls(std::vector<Control> &controls);

/**
 * @brief Appends controls with a straight line.
 * @param[in] q1 First configuration.
 * @param[in] q2 Second configuration.
 * @param[out] controls Sequence of path-segment controls.
 */
void StraightControls(const Configuration &q1, const Configuration &q2, std::vector<Control> &controls);

/**
 * @brief Appends controls with a rs-turn.
 * @param[out] c HC/CC circle.
 * @param[in] q Configuration on the circle or path.
 * @param[in] order True to append the turn in forward order.
 * @param[out] controls Sequence of path-segment controls.
 */
void RsTurnControls(const HcCcCircle &c, const Configuration &q, bool order, std::vector<Control> &controls);

/**
 * @brief Appends controls with a hc-turn.
 * @param[out] c HC/CC circle.
 * @param[in] q Configuration on the circle or path.
 * @param[in] order True to append the turn in forward order.
 * @param[out] controls Sequence of path-segment controls.
 */
void HcTurnControls(const HcCcCircle &c, const Configuration &q, bool order, std::vector<Control> &controls);

/**
 * @brief Appends controls with an elementary path if one exists.
 * @param[out] c HC/CC circle.
 * @param[in] q Configuration on the circle or path.
 * @param[in] delta Deflection angle in radians.
 * @param[in] order True to append the turn in forward order.
 * @param[out] controls Sequence of path-segment controls.
 * @return True when the predicate holds.
 */
bool CcElementaryControls(const HcCcCircle &c, const Configuration &q, double delta, bool order,
                            std::vector<Control> &controls);

/**
 * @brief Appends controls with a default cc-turn consisting of two clothoids and a circular arc.
 * @param[out] c HC/CC circle.
 * @param[in] q Configuration on the circle or path.
 * @param[in] delta Deflection angle in radians.
 * @param[in] order True to append the turn in forward order.
 * @param[out] controls Sequence of path-segment controls.
 */
void CcDefaultControls(const HcCcCircle &c, const Configuration &q, double delta, bool order,
                         std::vector<Control> &controls);

/**
 * @brief Appends controls with a cc-turn.
 * @param[out] c HC/CC circle.
 * @param[in] q Configuration on the circle or path.
 * @param[in] order True to append the turn in forward order.
 * @param[out] controls Sequence of path-segment controls.
 */
void CcTurnControls(const HcCcCircle &c, const Configuration &q, bool order, std::vector<Control> &controls);

}  // namespace steering
}  // namespace math
}  // namespace common
}  // namespace autonomy
