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
 * @file hybrid.cpp
 * @brief Hybrid-curvature Reeds–Shepp with arbitrary endpoint curvature (implementation).
 */

#include <algorithm>
#include <cmath>
#include <limits>

#include "autonomy/common/math/steering/clothoid/hybrid.hpp"
#include "autonomy/common/math/steering/utilities.hpp"

using namespace std;

namespace autonomy {
namespace common {
namespace math {
namespace steering {


HcReedsSheppStateSpace::HcReedsSheppStateSpace(double kappa, double sigma, double discretization)
  : HcCcStateSpace(kappa, sigma, discretization)
  , hc00_reeds_shepp_state_space_(kappa, sigma, discretization)
  , hc0pm_reeds_shepp_state_space_(kappa, sigma, discretization)
  , hcpm0_reeds_shepp_state_space_(kappa, sigma, discretization)
  , hcpmpm_reeds_shepp_state_space_(kappa, sigma, discretization)
{
}

vector<pair<State, Control>> HcReedsSheppStateSpace::PredictState(const State &state) const
{
  vector<pair<State, Control>> states_controls;

  // no prediction required
  if ((fabs(state.kappa) < GetEpsilon()) || ((kappa_ - fabs(state.kappa)) < GetEpsilon()))
  {
    pair<State, Control> state_control;
    state_control.first = state;
    state_control.second.delta_s = 0.0;
    state_control.second.kappa = state.kappa;
    state_control.second.sigma = 0.0;
    states_controls.push_back(state_control);
    return states_controls;
  }

  states_controls.reserve(4);
  double sgn_kappa = SignOfNumber(state.kappa);
  pair<State, Control> state_control1, state_control2, state_control3, state_control4;

  // assign controls
  state_control1.second.delta_s = (kappa_ - sgn_kappa * state.kappa) / sigma_;
  state_control1.second.kappa = state.kappa;
  state_control1.second.sigma = sgn_kappa * sigma_;
  states_controls.push_back(state_control1);

  state_control2.second.delta_s = -state_control1.second.delta_s;
  state_control2.second.kappa = state.kappa;
  state_control2.second.sigma = state_control1.second.sigma;
  states_controls.push_back(state_control2);

  state_control3.second.delta_s = sgn_kappa * state.kappa / sigma_;
  state_control3.second.kappa = state.kappa;
  state_control3.second.sigma = -sgn_kappa * sigma_;
  states_controls.push_back(state_control3);

  state_control4.second.delta_s = -state_control3.second.delta_s;
  state_control4.second.kappa = state.kappa;
  state_control4.second.sigma = state_control3.second.sigma;
  states_controls.push_back(state_control4);

  // predict states with controls
  for (auto &state_control : states_controls)
  {
    double d = SignOfNumber(state_control.second.delta_s);
    double abs_delta_s = fabs(state_control.second.delta_s);
    double sigma = state_control.second.sigma;
    EndOfClothoid(state.x, state.y, state.theta, state.kappa, sigma, d, abs_delta_s, &state_control.first.x,
                    &state_control.first.y, &state_control.first.theta, &state_control.first.kappa);
  }
  return states_controls;
}

double HcReedsSheppStateSpace::GetDistance(const State &state1, const State &state2) const
{
  vector<pair<State, Control>> start_states_controls = this->PredictState(state1);
  vector<pair<State, Control>> end_states_controls = this->PredictState(state2);
  vector<double> distances;
  distances.reserve(16);

  // compute the path length for all predicted start and end states
  for (const auto &start_state_control : start_states_controls)
  {
    State start_state = start_state_control.first;
    Control start_control = start_state_control.second;
    for (const auto &end_state_control : end_states_controls)
    {
      State end_state = end_state_control.first;
      Control end_control = end_state_control.second;
      // check if start and goal state are equal
      if (StateEqual(start_state, end_state))
      {
        Control control = SubtractControl(start_control, end_control);
        distances.push_back(fabs(control.delta_s));
      }
      // call appropriate state space
      else
      {
        double distance = 0.0;
        if (fabs(start_state.kappa) < GetEpsilon())
        {
          if (fabs(end_state.kappa) < GetEpsilon())
            distance += hc00_reeds_shepp_state_space_.GetDistance(start_state, end_state);
          else
            distance += hc0pm_reeds_shepp_state_space_.GetDistance(start_state, end_state);
        }
        else
        {
          if (fabs(end_state.kappa) < GetEpsilon())
            distance += hcpm0_reeds_shepp_state_space_.GetDistance(start_state, end_state);
          else
            distance += hcpmpm_reeds_shepp_state_space_.GetDistance(start_state, end_state);
        }
        // adjust controls by intial and final control
        if (fabs(start_control.delta_s) > GetEpsilon())
          distance += fabs(start_control.delta_s);
        if (fabs(end_control.delta_s) > GetEpsilon())
          distance += fabs(end_control.delta_s);
        distances.push_back(distance);
      }
    }
  }
  return *min_element(distances.begin(), distances.end());
}

vector<Control> HcReedsSheppStateSpace::GetControls(const State &state1, const State &state2) const
{
  vector<pair<State, Control>> start_states_controls = this->PredictState(state1);
  vector<pair<State, Control>> end_states_controls = this->PredictState(state2);
  vector<pair<vector<Control>, double>> hc_rs_controls_distance_pairs;
  hc_rs_controls_distance_pairs.reserve(16);

  // compute the path for all predicted start and end states
  for (const auto &start_state_control : start_states_controls)
  {
    State start_state = start_state_control.first;
    Control start_control = start_state_control.second;
    for (const auto &end_state_control : end_states_controls)
    {
      State end_state = end_state_control.first;
      Control end_control = end_state_control.second;
      vector<Control> hc_rs_controls;
      // check if start and goal state are equal
      if (StateEqual(start_state, end_state))
      {
        Control control = SubtractControl(start_control, end_control);
        hc_rs_controls.push_back(control);
      }
      // call the appropriate state space
      else
      {
        if (fabs(start_state.kappa) < GetEpsilon())
        {
          if (fabs(end_state.kappa) < GetEpsilon())
            hc_rs_controls = hc00_reeds_shepp_state_space_.GetControls(start_state, end_state);
          else
            hc_rs_controls = hc0pm_reeds_shepp_state_space_.GetControls(start_state, end_state);
        }
        else
        {
          if (fabs(end_state.kappa) < GetEpsilon())
            hc_rs_controls = hcpm0_reeds_shepp_state_space_.GetControls(start_state, end_state);
          else
            hc_rs_controls = hcpmpm_reeds_shepp_state_space_.GetControls(start_state, end_state);
        }
        // adjust controls by intial and final control
        if (fabs(start_control.delta_s) > GetEpsilon())
        {
          hc_rs_controls.insert(hc_rs_controls.begin(), start_control);
        }
        if (fabs(end_control.delta_s) > GetEpsilon())
        {
          ReverseControl(end_control);
          hc_rs_controls.insert(hc_rs_controls.end(), end_control);
        }
      }

      // compute the path length
      double distance = 0.0;
      for (const auto &control : hc_rs_controls)
        distance += fabs(control.delta_s);

      // push back
      pair<vector<Control>, double> hc_rs_controls_distance_pair;
      hc_rs_controls_distance_pair.first = hc_rs_controls;
      hc_rs_controls_distance_pair.second = distance;
      hc_rs_controls_distance_pairs.push_back(hc_rs_controls_distance_pair);
    }
  }

  // sort the controls with respect to path length
  sort(hc_rs_controls_distance_pairs.begin(), hc_rs_controls_distance_pairs.end(),
       [](const pair<vector<Control>, double> &i, const pair<vector<Control>, double> &j) {
         return i.second < j.second;
       });

  return hc_rs_controls_distance_pairs[0].first;
}

}  // namespace steering
}  // namespace math
}  // namespace common
}  // namespace autonomy
