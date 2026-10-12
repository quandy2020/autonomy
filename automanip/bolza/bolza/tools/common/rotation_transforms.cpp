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

#include <bolza/tools/common/rotation_transforms.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Eigen::Quaternion<ad_scalar_t> MatrixToQuaternion(const Eigen::Matrix<ad_scalar_t, 3, 3>& R) {
  ad_scalar_t t1, t2, t;
  ad_scalar_t x1, x2, x;
  ad_scalar_t y1, y2, y;
  ad_scalar_t z1, z2, z;
  ad_scalar_t w1, w2, w;

  t1 = CppAD::CondExpGt(R(0, 0), R(1, 1), 1 + R(0, 0) - R(1, 1) - R(2, 2), 1 - R(0, 0) + R(1, 1) - R(2, 2));
  t2 = CppAD::CondExpLt(R(0, 0), -R(1, 1), 1 - R(0, 0) - R(1, 1) + R(2, 2), 1 + R(0, 0) + R(1, 1) + R(2, 2));
  t = CppAD::CondExpLt(R(2, 2), ad_scalar_t(0.0), t1, t2);

  x1 = CppAD::CondExpGt(R(0, 0), R(1, 1), t, R(1, 0) + R(0, 1));
  x2 = CppAD::CondExpLt(R(0, 0), -R(1, 1), R(0, 2) + R(2, 0), R(2, 1) - R(1, 2));
  x = CppAD::CondExpLt(R(2, 2), ad_scalar_t(0.0), x1, x2);

  y1 = CppAD::CondExpGt(R(0, 0), R(1, 1), R(1, 0) + R(0, 1), t);
  y2 = CppAD::CondExpLt(R(0, 0), -R(1, 1), R(2, 1) + R(1, 2), R(0, 2) - R(2, 0));
  y = CppAD::CondExpLt(R(2, 2), ad_scalar_t(0.0), y1, y2);

  z1 = CppAD::CondExpGt(R(0, 0), R(1, 1), R(0, 2) + R(2, 0), R(2, 1) + R(1, 2));
  z2 = CppAD::CondExpLt(R(0, 0), -R(1, 1), t, R(1, 0) - R(0, 1));
  z = CppAD::CondExpLt(R(2, 2), ad_scalar_t(0.0), z1, z2);

  w1 = CppAD::CondExpGt(R(0, 0), R(1, 1), R(2, 1) - R(1, 2), R(0, 2) - R(2, 0));
  w2 = CppAD::CondExpLt(R(0, 0), -R(1, 1), R(1, 0) - R(0, 1), t);
  w = CppAD::CondExpLt(R(2, 2), ad_scalar_t(0.0), w1, w2);

  Eigen::Matrix<ad_scalar_t, 4, 1> q({x, y, z, w});
  q *= 0.5 / sqrt(t);

  return Eigen::Quaternion<ad_scalar_t>(q(3), q(0), q(1), q(2));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
namespace {  // helper functions to select the right modulo
template <typename SCALAR_T>
SCALAR_T scalarMod(SCALAR_T, SCALAR_T);

template <>
float scalarMod<float>(float x, float y) {
  return fmodf(x, y);
}

template <>
double scalarMod<double>(double x, double y) {
  return fmod(x, y);
}
}  // namespace

scalar_t ModuloAngleWithReference(scalar_t x, scalar_t reference) {
  const scalar_t ub = reference + M_PI;  // upper bound
  const scalar_t lb = reference - M_PI;  // lower bound

  if (x > ub) {
    x = lb + scalarMod(x - lb, 2.0 * M_PI);
  } else if (x < lb) {
    x = ub - scalarMod(ub - x, 2.0 * M_PI);
  }

  return x;
}

}  // namespace bolza
