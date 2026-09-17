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

// Ported into autonomy::control::tools
// Simple sequential filter chain (no FilterBase / pluginlib).

#ifndef AUTONOMY_CONTROL_TOOLS__FILTER_CHAIN_HPP_
#define AUTONOMY_CONTROL_TOOLS__FILTER_CHAIN_HPP_

#include <functional>
#include <utility>
#include <vector>

#include "autonomy/control/tools/exponential_filter.hpp"

namespace autonomy {
namespace control {
namespace tools {

/**
 * \brief Ordered list of scalar filters applied sequentially.
 *
 * Stages may be `ExponentialFilter<double>` instances (via `add_exponential`)
 * or arbitrary `std::function<double(double)>` callables.
 */
template <typename T = double>
class FilterChain
{
public:
  FilterChain() = default;

  std::size_t size() const { return stages_.size(); }

  /** \brief Append an exponential (EMA) stage with the given alpha. */
  void add_exponential(double alpha)
  {
    exp_filters_.emplace_back(alpha);
    stages_.emplace_back(StageKind::Exponential, exp_filters_.size() - 1);
  }

  /** \brief Append a custom callable stage. */
  void add(std::function<T(T)> fn)
  {
    callables_.emplace_back(std::move(fn));
    stages_.emplace_back(StageKind::Callable, callables_.size() - 1);
  }

  /** \brief Run the full chain; returns the final filtered value. */
  T update(T in)
  {
    T value = in;
    for (const auto & stage : stages_)
    {
      if (stage.kind == StageKind::Exponential)
      {
        value = exp_filters_[stage.index].update(value);
      }
      else
      {
        value = callables_[stage.index](value);
      }
    }
    last_ = value;
    return value;
  }

  /** \brief Reset all exponential stages; callables are left as-is. */
  void reset()
  {
    for (auto & f : exp_filters_)
    {
      f.reset();
    }
    last_ = T{};
  }

  const T & last() const { return last_; }

  void clear()
  {
    stages_.clear();
    exp_filters_.clear();
    callables_.clear();
    last_ = T{};
  }

private:
  enum class StageKind
  {
    Exponential,
    Callable
  };

  struct Stage
  {
    Stage(StageKind k, std::size_t i) : kind(k), index(i) {}
    StageKind kind;
    std::size_t index;
  };

  std::vector<Stage> stages_;
  std::vector<ExponentialFilter<T>> exp_filters_;
  std::vector<std::function<T(T)>> callables_;
  T last_{};
};

}  // namespace tools
}  // namespace control
}  // namespace autonomy

#endif  // AUTONOMY_CONTROL_TOOLS__FILTER_CHAIN_HPP_
