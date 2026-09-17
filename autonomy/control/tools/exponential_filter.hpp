// Copyright (c) 2024, AIT Austrian Institute of Technology GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Ported into autonomy::control::tools (standalone; no FilterBase / pluginlib)

#ifndef AUTONOMY_CONTROL_TOOLS__EXPONENTIAL_FILTER_HPP_
#define AUTONOMY_CONTROL_TOOLS__EXPONENTIAL_FILTER_HPP_

#include "autonomy/control/tools/filters.hpp"

namespace autonomy {
namespace control {
namespace tools {

/**
 * \brief Standalone exponential (EMA) smoother.
 *
 * Holds alpha and the last smoothed value. Uses `exponentialSmoothing` from
 * `filters.hpp`. Alpha is typically in (0, 1]; values closer to 0 weight the
 * previous output more heavily.
 *
 * Primary use is `T = double` (same as upstream control_filters).
 */
template <typename T = double>
class ExponentialFilter
{
public:
  ExponentialFilter() = default;

  explicit ExponentialFilter(double alpha) : alpha_(alpha) {}

  void set_alpha(double alpha) { alpha_ = alpha; }

  double alpha() const { return alpha_; }

  /** \brief Reset so the next `update` seeds from the input. */
  void reset()
  {
    initialized_ = false;
    last_ = T{};
  }

  /**
   * \brief One filter step.
   * \param in  Raw sample
   * \return Smoothed value
   */
  T update(T in)
  {
    if (!initialized_)
    {
      last_ = in;
      initialized_ = true;
      return last_;
    }
    last_ = static_cast<T>(
      exponentialSmoothing(static_cast<double>(in), static_cast<double>(last_), alpha_));
    return last_;
  }

  const T & last() const { return last_; }

  bool initialized() const { return initialized_; }

private:
  double alpha_{0.0};
  T last_{};
  bool initialized_{false};
};

}  // namespace tools
}  // namespace control
}  // namespace autonomy

#endif  // AUTONOMY_CONTROL_TOOLS__EXPONENTIAL_FILTER_HPP_
