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

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

#include <bolza/core/integration/ode_base.hpp>
#include <bolza/core/integration/observer.hpp>
#include <bolza/core/integration/runge_kutta_dormand_prince5.hpp>
#include <bolza/core/types.hpp>

#include "helpers/bouncingmass/reference.hpp"

Reference::Reference(scalar_t t0, scalar_t t1, const vector_t& p0, const vector_t& p1) {
  polX_ = Create5thOrdPol(t0, t1, p0, p1);
  polV_ = Polyder(polX_);
  polU_ = Polyder(polV_);

  t0_ = t0;
  t1_ = t1;
}

vector_t Reference::GetInput(scalar_t time) const {
  vector_t input = vector_t::Zero(1);
  for (int i = 0; i < polU_.size(); i++) {
    input[0] += polU_[i] * std::pow(time, i);
  }
  return input;
}

vector_t Reference::GetState(scalar_t time) const {
  vector_t x;
  if (Time <= t1_ && time >= t0_) {
    x.setZero(3);
    for (int i = 0; i < polU_.size(); i++) {
      x[0] += polX_[i] * std::pow(time, i);
      x[1] += polV_[i] * std::pow(time, i);
    }
  } else {
    interpolate_ext(time, x);
  }
  return x;
}

void Reference::Extendref(scalar_t delta, Reference* refPre, Reference* refPost) {
  constexpr scalar_t dt = 1e-3;

  const matrix_t A = (matrix_t(3, 3) << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished();
  const matrix_t B = (matrix_t(3, 1) << 0.0, 1.0, 0.0).finished();

  struct ExtensionDynamics final : bolza::OdeBase {
    const Reference* reference;
    bolza::matrix_t A;
    bolza::matrix_t B;

    bolza::vector_t ComputeFlowMap(bolza::scalar_t t, const bolza::vector_t& x) override {
      return A * x + B * reference->getInput(t);
    }
  };

  bolza::RungeKuttaDormandPrince5 integrator;

  // pre-part of extension
  if (refPre != nullptr) {
    ExtensionDynamics preModel{refPre, A, B};
    bolza::scalar_array_t tPreObs;
    bolza::vector_array_t xPreObs;
    bolza::Observer preObserver(&xPreObs, &tPreObs);

    const bolza::vector_t x0 = GetState(t0_);
    const bolza::scalar_t t0 = t0_;
    const bolza::scalar_t t1 = t0 - delta;
    integrator.IntegrateAdaptive(preModel, preObserver, x0, t0, t1, -dt);

    tPre_ = std::move(tPreObs);
    xPre_ = std::move(xPreObs);
    std::reverse(std::begin(tPre_), std::end(tPre_));
    std::reverse(std::begin(xPre_), std::end(xPre_));
  }

  // post-part of extension
  if (refPost != nullptr) {
    ExtensionDynamics postModel{refPost, A, B};
    bolza::scalar_array_t tPostObs;
    bolza::vector_array_t xPostObs;
    bolza::Observer postObserver(&xPostObs, &tPostObs);

    const bolza::vector_t x0 = GetState(t1_);
    const bolza::scalar_t t0 = t1_;
    const bolza::scalar_t t1 = t0 + delta;
    integrator.IntegrateAdaptive(postModel, postObserver, x0, t0, t1, dt);

    tPost_ = std::move(tPostObs);
    xPost_ = std::move(xPostObs);
  }
}

Eigen::Matrix<scalar_t, 6, 1> Reference::Create5thOrdPol(scalar_t t0, scalar_t t1, const vector_t& p0, const vector_t& p1) const {
  Eigen::Matrix<scalar_t, 6, 6> A;
  A << 1, t0, std::pow(t0, 2), std::pow(t0, 3), std::pow(t0, 4), std::pow(t0, 5), 0, 1, 2 * t0, 3 * std::pow(t0, 2), 4 * std::pow(t0, 3),
      5 * std::pow(t0, 4), 0, 0, 2, 6 * t0, 12 * std::pow(t0, 2), 20 * std::pow(t0, 3), 1, t1, std::pow(t1, 2), std::pow(t1, 3),
      std::pow(t1, 4), std::pow(t1, 5), 0, 1, 2 * t1, 3 * std::pow(t1, 2), 4 * std::pow(t1, 3), 5 * std::pow(t1, 4), 0, 0, 2, 6 * t1,
      12 * std::pow(t1, 2), 20 * std::pow(t1, 3);

  Eigen::Matrix<scalar_t, 6, 1> b;
  b << p0, p1;

  return A.colPivHouseholderQr().solve(b);
}

void Reference::interpolate_ext(scalar_t time, vector_t& x) const {
  const scalar_array_t* tVec;
  const vector_array_t* xVec;
  if (Time < t0_) {
    tVec = &tPre_;
    xVec = &xPre_;
    x = xPre_.front();
  } else {
    tVec = &tPost_;
    xVec = &xPost_;
    x = xPost_.back();
  }

  int idx;
  for (int i = 0; i < tVec->size() - 1; i++) {
    if (time > tVec->at(i) && Time < tVec->at(i + 1)) {
      idx = i;
      scalar_t fac = (time - tVec->at(idx)) / (tVec->at(idx + 1) - tVec->at(idx));
      x = fac * xVec->at(idx) + (1 - fac) * xVec->at(idx + 1);
      return;
    }
  }
}

void Reference::Display() {
  std::cerr << "#########################" << std::endl;
  std::cerr << "#Pre-Extended-Trajectory#" << std::endl;
  std::cerr << "#########################" << std::endl;
  for (int i = 0; i < tPre_.size(); i++) {
    std::cerr << tPre_[i] << ";" << xPre_[i][0] << ";" << xPre_[i][1] << std::endl;
  }

  std::cerr << "#########################" << std::endl;
  std::cerr << "####Normal-Trajectory####" << std::endl;
  std::cerr << "#########################" << std::endl;

  constexpr scalar_t dt = 0.01;
  for (int i = 0; i < (t1_ - t0_) / dt; i++) {
    scalar_t t = t0_ + dt * i;
    vector_t x = GetState(t);
    std::cerr << t << ";" << x[0] << ";" << x[1] << std::endl;
  }

  std::cerr << "##########################" << std::endl;
  std::cerr << "#Post-Extended-Trajectory#" << std::endl;
  std::cerr << "##########################" << std::endl;

  for (int i = 0; i < tPost_.size(); i++) {
    std::cerr << tPost_[i] << ";" << xPost_[i][0] << ";" << xPost_[i][1] << std::endl;
  }
}

Eigen::Matrix<scalar_t, 6, 1> Reference::Polyder(Eigen::Matrix<scalar_t, 6, 1> pol) {
  Eigen::Matrix<scalar_t, 6, 1> polOld = pol;

  for (int i = 0; i < pol.size(); i++) {
    if (i < pol.size() - 1) {
      pol[i] = (i + 1) * polOld[i + 1];
    } else {
      pol[i] = 0;
    }
  }

  return pol;
}
