/*********************************************************************
*  Copyright (c) 2017 Robert Bosch GmbH.
*  All rights reserved.
*
*  Licensed under the Apache License, Version 2.0 (the "License");
*  you may not use this file except in compliance with the License.
*  You may obtain a copy of the License at
*
*      http://www.apache.org/licenses/LICENSE-2.0
*
*  Unless required by applicable law or agreed to in writing, software
*  distributed under the License is distributed on an "AS IS" BASIS,
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*  See the License for the specific language governing permissions and
*  limitations under the License.
* *********************************************************************/

#include <cmath>

#include "gtest/gtest.h"

#include "autonomy/common/math/steering/covariance.hpp"
#include "autonomy/common/math/steering/types.hpp"
#include "autonomy/common/math/steering/utilities.hpp"

#define EPS_JACOBI 1e-4                  // [-]
#define EPS_PERTURB 1e-7                 // [-]
#define SAMPLES 256                      // [-]
#define OPERATING_REGION_X 20.0          // [m]
#define OPERATING_REGION_Y 20.0          // [m]
#define OPERATING_REGION_THETA 2 * M_PI  // [rad]
#define OPERATING_REGION_KAPPA 2.0       // [1/m]
#define OPERATING_REGION_DELTA_S 0.4     // [m]
#define OPERATING_REGION_SIGMA 2.0       // [1/m^2]
#define random(lower, upper) (rand() * (upper - lower) / RAND_MAX + lower)
#define random_boolean() rand() % 2

using namespace std;
using namespace autonomy::common::math::steering;

State GetRandomState()
{
  State state;
  state.x = random(-OPERATING_REGION_X / 2.0, OPERATING_REGION_X / 2.0);
  state.y = random(-OPERATING_REGION_Y / 2.0, OPERATING_REGION_Y / 2.0);
  state.theta = random(-OPERATING_REGION_THETA / 2.0, OPERATING_REGION_THETA / 2.0);
  state.kappa = random(-OPERATING_REGION_KAPPA / 2.0, OPERATING_REGION_KAPPA / 2.0);
  state.d = 0.0;
  return state;
}

Control GetRandomControl()
{
  Control control;
  control.delta_s = random(-OPERATING_REGION_DELTA_S / 2.0, OPERATING_REGION_DELTA_S / 2.0);
  control.sigma = random(-OPERATING_REGION_SIGMA / 2.0, OPERATING_REGION_SIGMA / 2.0);
  return control;
}

State IntegrateOde(const State &state, const Control &control, double integration_step)
{
  State state_next;
  double sigma(control.sigma);
  double d(SignOfNumber(control.delta_s));
  if (fabs(sigma) > GetEpsilon())
  {
    EndOfClothoid(state.x, state.y, state.theta, state.kappa, sigma, d, integration_step, &state_next.x,
                    &state_next.y, &state_next.theta, &state_next.kappa);
  }
  else
  {
    if (fabs(state.kappa) > GetEpsilon())
    {
      EndOfCircularArc(state.x, state.y, state.theta, state.kappa, d, integration_step, &state_next.x, &state_next.y,
                          &state_next.theta);
    }
    else
    {
      EndOfStraightLine(state.x, state.y, state.theta, d, integration_step, &state_next.x, &state_next.y);
    }
  }
  state_next.theta =
      NormalizeAngle(state.theta + state.kappa * d * integration_step + 0.5 * sigma * d * integration_step * integration_step);
  state_next.kappa = state.kappa + sigma * integration_step;
  state_next.d = d;

  return state_next;
}

Matrix3d GetNumMotionJacobiX(const State &state, const Control &control, double integration_step)
{
  Matrix3d f_x;
  Matrix3d perturb(EPS_PERTURB * Matrix3d::Identity());

  State next = IntegrateOde(state, control, integration_step);
  for (int i = 0; i < f_x.cols(); ++i)
  {
    State state_perturb;
    state_perturb.x = state.x + perturb(0, i);
    state_perturb.y = state.y + perturb(1, i);
    state_perturb.theta = state.theta + perturb(2, i);
    state_perturb.kappa = state.kappa;

    State next_perturb = IntegrateOde(state_perturb, control, integration_step);
    f_x(0, i) = (next_perturb.x - next.x) / perturb(i, i);
    f_x(1, i) = (next_perturb.y - next.y) / perturb(i, i);
    f_x(2, i) = (next_perturb.theta - next.theta) / perturb(i, i);
  }
  return f_x;
}

Matrix32d GetNumMotionJacobiU(const State &state, const Control &control, double integration_step)
{
  Matrix32d f_u;
  Matrix2d perturb(EPS_PERTURB * Matrix2d::Identity());

  State next = IntegrateOde(state, control, integration_step);
  for (int i = 0; i < f_u.cols(); ++i)
  {
    Control control_perturb;
    control_perturb.delta_s = control.delta_s + perturb(0, i);
    control_perturb.sigma = control.sigma;
    double integration_step_perturb = fabs(control_perturb.delta_s);

    State state_perturb;
    state_perturb.x = state.x;
    state_perturb.y = state.y;
    state_perturb.theta = state.theta;
    state_perturb.kappa = state.kappa + perturb(1, i);

    State next_perturb = IntegrateOde(state_perturb, control_perturb, integration_step_perturb);
    f_u(0, i) = (next_perturb.x - next.x) / perturb(i, i);
    f_u(1, i) = (next_perturb.y - next.y) / perturb(i, i);
    f_u(2, i) = (next_perturb.theta - next.theta) / perturb(i, i);
  }
  return f_u;
}

TEST(Jacobian, f_x)
{
  for (int i = 0; i < SAMPLES; i++)
  {
    State state = GetRandomState();
    Control control = GetRandomControl();

    state.kappa = random_boolean() * state.kappa;
    control.sigma = random_boolean() * control.sigma;
    double integration_step = fabs(control.delta_s);

    Matrix3d f_x_ana(Matrix3d::Zero());
    Matrix32d f_u_ana(Matrix32d::Zero());
    MotionJacobian(state, control, integration_step, f_x_ana, f_u_ana);
    Matrix3d f_x_num = GetNumMotionJacobiX(state, control, integration_step);
    for (int i = 0; i < f_x_ana.rows(); ++i)
    {
      for (int j = 0; j < f_x_ana.cols(); ++j)
      {
        EXPECT_LE(fabs(f_x_ana(i, j) - f_x_num(i, j)), EPS_JACOBI);
      }
    }
  }
}

TEST(Jacobian, f_u)
{
  for (int i = 0; i < SAMPLES; i++)
  {
    State state = GetRandomState();
    Control control = GetRandomControl();

    state.kappa = random_boolean() * state.kappa;
    control.sigma = random_boolean() * control.sigma;
    double integration_step = fabs(control.delta_s);

    Matrix3d f_x_ana(Matrix3d::Zero());
    Matrix32d f_u_ana(Matrix32d::Zero());
    MotionJacobian(state, control, integration_step, f_x_ana, f_u_ana);
    Matrix32d f_u_num = GetNumMotionJacobiU(state, control, integration_step);
    for (int i = 0; i < f_u_ana.rows(); ++i)
    {
      for (int j = 0; j < f_u_ana.cols(); ++j)
      {
        EXPECT_LE(fabs(f_u_ana(i, j) - f_u_num(i, j)), EPS_JACOBI);
      }
    }
  }
}

