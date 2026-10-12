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
 * @file numerics.hpp
 * @brief Almost equal which uses machine epsilon to compare floating-point values for equality.
 */


#include <cmath>
#include <limits>
#include <type_traits>

namespace bolza {
namespace numerics {

/**
 * Almost equal which uses machine epsilon to compare floating-point values for equality.
 * refer to: https://en.cppreference.com/w/cpp/types/numeric_limits/epsilon
 *
 * @tparam T1: data type of x.
 * @tparam T2: data type of y.
 * @tparam T3: data type of prec.
 * @param [in] x: First floating-point number.
 * @param [in] y: Second floating-point number.
 * @param [in] prec: The comparison precision.
 * @return bool: true if x=y.
 */
template <class T1, class T2, class T3>
/**
 * @brief Almost eq.
 * @param [in,out] x: X.
 * @param [in,out] y: Y.
 * @param [in,out] prec: Prec.
 * @return True on success.
 */
bool almost_eq(T1&& x, T2&& y, T3&& prec) {
  static_assert(std::is_floating_point<typename std::remove_reference<T1>::type>::value, "First argument is not floating point!");
  static_assert(std::is_floating_point<typename std::remove_reference<T2>::type>::value, "Second argument is not floating point!");
  static_assert(std::is_floating_point<typename std::remove_reference<T3>::type>::value, "prec is not floating point!");
  // the machine epsilon has to be scaled to the magnitude of the values used
  // and multiplied by the desired precision unless the result is subnormal
  using Type = const std::remove_reference_t<T1>;
  /**
   * @brief Abs.
   * @return Result value.
   */
  const auto absDiff = std::abs(x - static_cast<Type>(y));
  /**
   * @brief Min.
   * @return Result value.
   */
  const auto magnitude = std::min(std::abs(x), std::abs(static_cast<Type>(y)));
  /**
   * @brief Min.
   * @return Result value.
   */
  return absDiff <= static_cast<Type>(prec) * magnitude || absDiff < std::numeric_limits<Type>::min();
}

/**
 * Almost equal which uses machine epsilon to compare floating-point values for equality.
 * refer to: https://en.cppreference.com/w/cpp/types/numeric_limits/epsilon
 *
 * @tparam T1: data type of x.
 * @tparam T2: data type of y.
 * @param [in] x: First floating-point number.
 * @param [in] y: Second floating-point number.
 * @return bool: true if x=y.
 */
template <class T1, class T2>
/**
 * @brief Almost eq.
 * @param [in,out] x: X.
 * @param [in,out] y: Y.
 * @return True on success.
 */
bool almost_eq(T1&& x, T2&& y) {
  /**
   * @brief Epsilon.
   * @return Result value.
   */
  const auto prec = std::numeric_limits<std::remove_reference_t<T1>>::epsilon();
  /**
   * @brief Almost eq.
   * @param [in] x: X.
   * @param [in] y: Y.
   * @param [in] prec: Prec.
   * @return Result value.
   */
  return almost_eq(x, y, prec);
}

/**
 * Almost less-equal which uses machine epsilon to compare floating-point values for equality.
 *
 * @tparam T1: data type of x.
 * @tparam T2: data type of y.
 * @tparam T3: data type of prec.
 * @param [in] x: First floating-point number.
 * @param [in] y: Second floating-point number.
 * @param [in] prec: The comparison precision.
 * @return bool: true if x<=y.
 */
template <class T1, class T2, class T3>
/**
 * @brief Almost le.
 * @param [in,out] x: X.
 * @param [in,out] y: Y.
 * @param [in,out] prec: Prec.
 * @return True on success.
 */
bool almost_le(T1&& x, T2&& y, T3&& prec) {
  /**
   * @brief Almost eq.
   * @param [in] x: X.
   * @param [in] y: Y.
   * @param [in] prec: Prec.
   * @return Result value.
   */
  return x < y || almost_eq(x, y, prec);
}

/**
 * Almost less-equal which uses machine epsilon to compare floating-point values for equality.
 *
 * @tparam T1: data type of x.
 * @tparam T2: data type of y.
 * @param [in] x: First floating-point number.
 * @param [in] y: Second floating-point number.
 * @return bool: true if x<=y.
 */
template <class T1, class T2, class T3>
/**
 * @brief Almost le.
 * @param [in,out] x: X.
 * @param [in,out] y: Y.
 * @return True on success.
 */
bool almost_le(T1&& x, T2&& y) {
  /**
   * @brief Almost eq.
   * @param [in] x: X.
   * @param [in] y: Y.
   * @return Result value.
   */
  return x < y || almost_eq(x, y);
}

/**
 * Almost greater-equal which uses machine epsilon to compare floating-point values for equality.
 *
 * @tparam T1: data type of x.
 * @tparam T2: data type of y.
 * @tparam T3: data type of prec.
 * @param [in] x: First floating-point number.
 * @param [in] y: Second floating-point number.
 * @param [in] prec: The comparison precision.
 * @return bool: true if x>=y.
 */
template <class T1, class T2, class T3>
/**
 * @brief Almost ge.
 * @param [in,out] x: X.
 * @param [in,out] y: Y.
 * @param [in,out] prec: Prec.
 * @return True on success.
 */
bool almost_ge(T1&& x, T2&& y, T3&& prec) {
  /**
   * @brief Almost eq.
   * @param [in] x: X.
   * @param [in] y: Y.
   * @param [in] prec: Prec.
   * @return Result value.
   */
  return x > y || almost_eq(x, y, prec);
}

/**
 * Almost greater-equal which uses machine epsilon to compare floating-point values for equality.
 *
 * @tparam T1: data type of x.
 * @tparam T2: data type of y.
 * @param [in] x: First floating-point number.
 * @param [in] y: Second floating-point number.
 * @return bool: true if x>=y.
 */
template <class T1, class T2>
/**
 * @brief Almost ge.
 * @param [in,out] x: X.
 * @param [in,out] y: Y.
 * @return True on success.
 */
bool almost_ge(T1&& x, T2&& y) {
  /**
   * @brief Almost eq.
   * @param [in] x: X.
   * @param [in] y: Y.
   * @return Result value.
   */
  return x > y || almost_eq(x, y);
}

}  // namespace numerics
}  // namespace bolza
