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

#ifndef AUTONOMY_COMMON_MATH_MATH_HPP_
#define AUTONOMY_COMMON_MATH_MATH_HPP_

#include <cmath>
#include <vector>
#include <algorithm>
#include <complex>
#include <limits>
#include <list>
#include <stdexcept>

#include "Eigen/Core"

#include "autonomy/common/port.hpp"
#include "autonomy/common/logging.hpp"
// #include "ceres/ceres.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288
#endif

namespace autonomy {
namespace common {

// Calculates 'base'^'exponent'.
template <typename T>
constexpr T Power(T base, int exponent) {
  return (exponent != 0) ? base * Power(base, exponent - 1) : T(1);
}

// Calculates a^2.
template <typename T>
constexpr T Pow2(T a) {
  return Power(a, 2);
}

// Converts from degrees to radians.
constexpr double DegToRad(double deg) { return M_PI * deg / 180.; }

// Converts form radians to degrees.
constexpr double RadToDeg(double rad) { return 180. * rad / M_PI; }

// Bring the 'difference' between two angles into [-pi; pi].
template <typename T>
T NormalizeAngleDifference(T difference) {
  const T kPi = T(M_PI);
  while (difference > kPi) difference -= 2. * kPi;
  while (difference < -kPi) difference += 2. * kPi;
  return difference;
}

// template <typename T>
// T atan2(const Eigen::Matrix<T, 2, 1>& vector) {
//   return ceres::atan2(vector.y(), vector.x());
// }

template <typename T>
inline void QuaternionProduct(const double* const z, const T* const w,
                              T* const zw) {
  zw[0] = z[0] * w[0] - z[1] * w[1] - z[2] * w[2] - z[3] * w[3];
  zw[1] = z[0] * w[1] + z[1] * w[0] + z[2] * w[3] - z[3] * w[2];
  zw[2] = z[0] * w[2] - z[1] * w[3] + z[2] * w[0] + z[3] * w[1];
  zw[3] = z[0] * w[3] + z[1] * w[2] - z[2] * w[1] + z[3] * w[0];
}

// Return 1 if number is positive, -1 if negative, and 0 if the number is 0.
template <typename T>
int SignOfNumber(T val);

// Clamp the given value to a low and maximum value.
template <typename T>
inline T Clamp(const T& value, const T& low, const T& high);

// Determine median value in vector. Returns NaN for empty vectors.
template <typename T>
double Median(const std::vector<T>& elems);

// Determine mean value in a vector.
template <typename T>
double Mean(const std::vector<T>& elems);

// Determine sample variance in a vector.
template <typename T>
double Variance(const std::vector<T>& elems);

// Determine sample standard deviation in a vector.
template <typename T>
double StdDev(const std::vector<T>& elems);

// Generate N-choose-K combinations.
//
// Note that elements in range [first, last) must be in sorted order,
// according to `std::less`.
template <class Iterator>
bool NextCombination(Iterator first, Iterator middle, Iterator last);

// Sigmoid function.
template <typename T>
T Sigmoid(T x, T alpha = 1);

// Scale values according to sigmoid transform.
//
//   x \in [0, 1] -> x \in [-x0, x0] -> sigmoid(x, alpha) -> x \in [0, 1]
//
// @param x        Value to be scaled in the range [0, 1].
// @param x0       Spread that determines the range x is scaled to.
// @param alpha    Exponential sigmoid factor.
//
// @return         The scaled value in the range [0, 1].
template <typename T>
T ScaleSigmoid(T x, T alpha = 1, T x0 = 10);

// Binomial coefficient or all combinations, defined as n! / ((n - k)! k!).
uint64_t NChooseK(uint64_t n, uint64_t k);

// Cast value from one type to another and truncate instead of overflow, if the
// input value is out of range of the output data type.
template <typename T1, typename T2>
T2 TruncateCast(T1 value);

// Compute the n-th percentile in the given sequence.
template <typename T>
T Percentile(const std::vector<T>& elems, double p);

////////////////////////////////////////////////////////////////////////////////
// Implementation
////////////////////////////////////////////////////////////////////////////////

namespace internal {

template <class Iterator>
bool NextCombination(Iterator first1,
                     Iterator last1,
                     Iterator first2,
                     Iterator last2) {
  if ((first1 == last1) || (first2 == last2)) {
    return false;
  }
  Iterator m1 = last1;
  Iterator m2 = last2;
  --m2;
  while (--m1 != first1 && *m1 >= *m2) {
  }
  bool result = (m1 == first1) && *first1 >= *m2;
  if (!result) {
    while (first2 != m2 && *m1 >= *first2) {
      ++first2;
    }
    first1 = m1;
    std::iter_swap(first1, first2);
    ++first1;
    ++first2;
  }
  if ((first1 != last1) && (first2 != last2)) {
    m1 = last1;
    m2 = first2;
    while ((m1 != first1) && (m2 != last2)) {
      std::iter_swap(--m1, m2);
      ++m2;
    }
    std::reverse(first1, m1);
    std::reverse(first1, last1);
    std::reverse(m2, last2);
    std::reverse(first2, last2);
  }
  return !result;
}

}  // namespace internal

template <typename T>
int SignOfNumber(const T val) {
  return (T(0) < val) - (val < T(0));
}

template <typename T>
T Clamp(const T& value, const T& low, const T& high) {
  return std::max(low, std::min(value, high));
}

template <typename T>
double Median(const std::vector<T>& elems) {
  THROW_CHECK(!elems.empty());

  const size_t mid_idx = elems.size() / 2;

  std::vector<T> ordered_elems = elems;
  std::nth_element(ordered_elems.begin(),
                   ordered_elems.begin() + mid_idx,
                   ordered_elems.end());

  if (elems.size() % 2 == 0) {
    const T mid_element1 = ordered_elems[mid_idx];
    const T mid_element2 = *std::max_element(ordered_elems.begin(),
                                             ordered_elems.begin() + mid_idx);
    return 0.5 * mid_element1 + 0.5 * mid_element2;
  } else {
    return ordered_elems[mid_idx];
  }
}

template <typename T>
T Percentile(const std::vector<T>& elems, const double p) {
  THROW_CHECK(!elems.empty());
  THROW_CHECK_GE(p, 0);
  THROW_CHECK_LE(p, 100);

  const int idx = static_cast<int>(std::round(p / 100 * (elems.size() - 1)));
  const size_t percentile_idx =
      std::max(0, std::min(static_cast<int>(elems.size() - 1), idx));

  std::vector<T> ordered_elems = elems;
  std::nth_element(ordered_elems.begin(),
                   ordered_elems.begin() + percentile_idx,
                   ordered_elems.end());

  return ordered_elems.at(percentile_idx);
}

template <typename T>
double Mean(const std::vector<T>& elems) {
  THROW_CHECK(!elems.empty());
  double sum = 0;
  for (const auto el : elems) {
    sum += static_cast<double>(el);
  }
  return sum / elems.size();
}

template <typename T>
double Variance(const std::vector<T>& elems) {
  const double mean = Mean(elems);
  double var = 0;
  for (const auto el : elems) {
    const double diff = el - mean;
    var += diff * diff;
  }
  return var / (elems.size() - 1);
}

template <typename T>
double StdDev(const std::vector<T>& elems) {
  return std::sqrt(Variance(elems));
}

template <class Iterator>
bool NextCombination(Iterator first, Iterator middle, Iterator last) {
  return internal::NextCombination(first, middle, middle, last);
}

template <typename T>
T Sigmoid(const T x, const T alpha) {
  return T(1) / (T(1) + std::exp(-x * alpha));
}

template <typename T>
T ScaleSigmoid(T x, const T alpha, const T x0) {
  const T t0 = Sigmoid(-x0, alpha);
  const T t1 = Sigmoid(x0, alpha);
  x = (Sigmoid(2 * x0 * x - x0, alpha) - t0) / (t1 - t0);
  return x;
}

template <typename T1, typename T2>
T2 TruncateCast(const T1 value) {
  return static_cast<T2>(std::min(
      static_cast<T1>(std::numeric_limits<T2>::max()),
      std::max(static_cast<T1>(std::numeric_limits<T2>::min()), value)));
}

}  // namespace common
}  // namespace autonomy

#endif  // AUTONOMY_COMMON_MATH_MATH_HPP_
