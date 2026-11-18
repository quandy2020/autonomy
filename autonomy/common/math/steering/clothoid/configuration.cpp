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
 * @file configuration.cpp
 * @brief Pose-and-curvature configuration on a clothoid path (implementation).
 */

#include <cmath>
#include <iostream>

#include "autonomy/common/math/steering/clothoid/configuration.hpp"
#include "autonomy/common/math/steering/utilities.hpp"

using namespace std;

namespace autonomy {
namespace common {
namespace math {
namespace steering {


Configuration::Configuration(double _x, double _y, double _theta, double _kappa)
{
  x = _x;
  y = _y;
  theta = WrapAngle(_theta);
  kappa = _kappa;
}

void Configuration::Print(bool eol) const
{
  cout << "(" << x << ", " << y << ", " << theta << ", " << kappa << ")";
  if (eol)
  {
    cout << endl;
  }
}

double ConfigurationDistance(const Configuration &q1, const Configuration &q2)
{
  return std::hypot((q2.x) - (q1.x), (q2.y) - (q1.y));
}

bool ConfigurationAligned(const Configuration &q1, const Configuration &q2)
{
  if (fabs(q2.theta - q1.theta) > GetEpsilon())
  {
    return false;
  }
  double angle = WrapAngle(atan2(q2.y - q1.y, q2.x - q1.x));
  return fabs(angle - q1.theta) <= GetEpsilon();
}

bool ConfigurationEqual(const Configuration &q1, const Configuration &q2)
{
  if (fabs(q2.theta - q1.theta) > GetEpsilon())
    return false;
  if (ConfigurationDistance(q1, q2) > GetEpsilon())
    return false;
  return true;
}

}  // namespace steering
}  // namespace math
}  // namespace common
}  // namespace autonomy
