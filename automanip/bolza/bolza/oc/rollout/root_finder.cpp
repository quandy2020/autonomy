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

#include <bolza/oc/rollout/root_finder.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void RootFinder::SetInitBracket(pair_t timeInt, pair_t guardInt) {
  if (guardInt.first * guardInt.second > 0) {
    throw std::runtime_error("Bracket function values should have opposite signs!");
  }
  timeInt_ = timeInt;
  guardInt_ = guardInt;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void RootFinder::UpdateBracket(const scalar_t& query, const scalar_t& fQuery) {
  if (fQuery * guardInt_.first < 0) {
    guardInt_.second = guardInt_.first;
    timeInt_.second = timeInt_.first;

    guardInt_.first = fQuery;
    timeInt_.first = query;

  } else {
    scalar_t gamma;
    switch (rootFindingAlgorithm_) {
      case (RootFinderType::ANDERSON_BJORCK): {
        gamma = 1 - (fQuery / guardInt_.first);
        if (gamma < 0) {
          gamma = 0.5;
        }
        break;
      }
      case (RootFinderType::PEGASUS): {
        gamma = guardInt_.first / (guardInt_.first + fQuery);
        break;
      }
      case (RootFinderType::ILLINOIS): {
        gamma = 0.5;
        break;
      }
      case (RootFinderType::REGULA_FALSI): {
        gamma = 1.0;
        break;
      }
      default: {
        throw std::runtime_error("Root finding algorithm of type " +
                                 std::to_string(static_cast<std::underlying_type<RootFinderType>::type>(rootFindingAlgorithm_)) +
                                 " is not supported.");
      }
    }

    guardInt_.first = fQuery;
    timeInt_.first = query;

    guardInt_.second *= gamma;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void RootFinder::Display() {
  std::cerr << "Root Finding Information\n";
  std::cerr << "Time Bracket:  [" << timeInt_.first << ";" << timeInt_.second << "]\n";
  std::cerr << "Value Bracket: [" << guardInt_.first << ";" << guardInt_.second << "]\n";
}
}  // namespace bolza
