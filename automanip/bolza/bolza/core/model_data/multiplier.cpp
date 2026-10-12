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

#include "bolza/core/model_data/multiplier.hpp"

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t ToVector(const std::vector<Multiplier>& termsMultiplier) {
  size_t n = 0;
  std::for_each(termsMultiplier.begin(), termsMultiplier.end(), [&](const Multiplier& m) { n += (1 + m.lagrangian.size()); });

  vector_t vec(n);
  size_t head = 0;
  for (const auto& m : termsMultiplier) {
    vec(head) = m.penalty;
    vec.segment(head + 1, m.lagrangian.size()) = m.lagrangian;
    head += 1 + m.lagrangian.size();
  }  // end of i loop

  return vec;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::vector<Multiplier> ToMultipliers(const size_array_t& termsSize, const vector_t& vec) {
  std::vector<Multiplier> multipliers;
  multipliers.reserve(termsSize.size());

  size_t head = 0;
  for (const auto& l : termsSize) {
    multipliers.emplace_back(vec(head), vec.segment(head + 1, l));
    head += 1 + l;
  }  // end of i loop

  return multipliers;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_array_t GetSizes(const std::vector<Multiplier>& termsMultiplier) {
  size_array_t s(termsMultiplier.size());
  std::transform(termsMultiplier.begin(), termsMultiplier.end(), s.begin(),
                 [](const Multiplier& m) { return static_cast<size_t>(m.lagrangian.size()); });
  return s;
}

}  // namespace bolza

namespace bolza {
namespace LinearInterpolation {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Multiplier Interpolate(const index_alpha_t& indexAlpha, const std::vector<MultiplierConstRef>& dataArray) {
  const auto penalty = Interpolate(
      indexAlpha, dataArray, [](const std::vector<MultiplierConstRef>& array, size_t t) -> const scalar_t& { return array[t].penalty; });

  const auto lagrangian = Interpolate(
      indexAlpha, dataArray, [](const std::vector<MultiplierConstRef>& array, size_t t) -> const vector_t& { return array[t].lagrangian; });

  return {penalty, lagrangian};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MultiplierCollection Interpolate(const index_alpha_t& indexAlpha, const std::vector<MultiplierCollection>& dataArray) {
  // number of terms
  const auto ind = indexAlpha.second > 0.5 ? indexAlpha.first : indexAlpha.first + 1;
  const size_t mumStateEq = dataArray[ind].stateEq.size();
  const size_t mumStateIneq = dataArray[ind].stateIneq.size();
  const size_t mumStateInputEq = dataArray[ind].stateInputEq.size();
  const size_t mumStateInputIneq = dataArray[ind].stateInputIneq.size();

  MultiplierCollection out;

  // state equality
  out.stateEq.reserve(mumStateEq);
  for (size_t i = 0; i < mumStateEq; i++) {
    auto penalty = Interpolate(indexAlpha, dataArray, [i](const std::vector<MultiplierCollection>& array, size_t t) -> const scalar_t& {
      return array[t].stateEq[i].penalty;
    });
    auto lagrangian = Interpolate(indexAlpha, dataArray, [i](const std::vector<MultiplierCollection>& array, size_t t) -> const vector_t& {
      return array[t].stateEq[i].lagrangian;
    });
    out.stateEq.emplace_back(penalty, std::move(lagrangian));
  }  // end of i loop

  // state inequality
  out.stateIneq.reserve(mumStateIneq);
  for (size_t i = 0; i < mumStateIneq; i++) {
    auto penalty = Interpolate(indexAlpha, dataArray, [i](const std::vector<MultiplierCollection>& array, size_t t) -> const scalar_t& {
      return array[t].stateIneq[i].penalty;
    });
    auto lagrangian = Interpolate(indexAlpha, dataArray, [i](const std::vector<MultiplierCollection>& array, size_t t) -> const vector_t& {
      return array[t].stateIneq[i].lagrangian;
    });
    out.stateIneq.emplace_back(penalty, std::move(lagrangian));
  }  // end of i loop

  // state-input equality
  out.stateInputEq.reserve(mumStateInputEq);
  for (size_t i = 0; i < mumStateInputEq; i++) {
    auto penalty = Interpolate(indexAlpha, dataArray, [i](const std::vector<MultiplierCollection>& array, size_t t) -> const scalar_t& {
      return array[t].stateInputEq[i].penalty;
    });
    auto lagrangian = Interpolate(indexAlpha, dataArray, [i](const std::vector<MultiplierCollection>& array, size_t t) -> const vector_t& {
      return array[t].stateInputEq[i].lagrangian;
    });
    out.stateInputEq.emplace_back(penalty, std::move(lagrangian));
  }  // end of i loop

  // state-input inequality
  out.stateInputIneq.reserve(mumStateInputIneq);
  for (size_t i = 0; i < mumStateInputIneq; i++) {
    auto penalty = Interpolate(indexAlpha, dataArray, [i](const std::vector<MultiplierCollection>& array, size_t t) -> const scalar_t& {
      return array[t].stateInputIneq[i].penalty;
    });
    auto lagrangian = Interpolate(indexAlpha, dataArray, [i](const std::vector<MultiplierCollection>& array, size_t t) -> const vector_t& {
      return array[t].stateInputIneq[i].lagrangian;
    });
    out.stateInputIneq.emplace_back(penalty, std::move(lagrangian));
  }  // end of i loop

  return out;
}

}  // namespace LinearInterpolation
}  // namespace bolza
