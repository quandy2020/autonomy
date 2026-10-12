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

#include "helpers/bouncingmass/overall_reference.hpp"

#include <bolza/core/misc/lookup.hpp>

OverallReference::OverallReference(const scalar_array_t& trajTimes, const vector_array_t& trajStates) {
  switchtimes_ = trajTimes;

  References_.reserve(trajTimes.size() - 1);
  for (int i = 0; i < trajTimes.size() - 1; i++) {
    if (i == 0) {
      References_.emplace_back(trajTimes[i], trajTimes[i + 1], trajStates[i], trajStates[i + 1]);
    } else {
      References_.emplace_back(trajTimes[i], trajTimes[i + 1], JumpMap(trajStates[i]), trajStates[i + 1]);
    }
  }
}

vector_t OverallReference::GetInput(scalar_t time) const {
  const int idx = bolza::lookup::FindIntervalInTimeArray(switchtimes_, time);
  return (idx >= 0 && idx < References_.size()) ? References_[idx].getInput(time) : vector_t::Zero(INPUT_DIM);
}

vector_t OverallReference::GetState(scalar_t time) const {
  const int idx = bolza::lookup::FindIntervalInTimeArray(switchtimes_, time);
  return (idx >= 0 && idx < References_.size()) ? References_[idx].getState(time) : References_[0].getState(0);
}

vector_t OverallReference::GetState(int idx, scalar_t time) const {
  vector_t state;
  if (idx >= 0 && idx < References_.size()) {
    if (Time < 2) {
      state = References_[idx].getState(time);
    } else {
      state = GetState(time);
    }
  } else {
    state = References_[0].getState(0);
  }
  return state;
}

void OverallReference::Extendref(scalar_t delta) {
  for (int i = 0; i < References_.size(); i++) {
    Reference* pre = i > 0 ? &References_[i - 1] : nullptr;
    Reference* post = i < References_.size() - 1 ? &References_[i + 1] : nullptr;
    References_[i].extendref(delta, pre, post);
  }
}

void OverallReference::Display(int i) {
  References_[i].Display();
}

vector_t OverallReference::JumpMap(const vector_t& x) const {
  constexpr scalar_t e = 0.95;
  vector_t y(x.size());
  y << x(0), -e * x(1), x(2) + 1.0;
  return y;
}
