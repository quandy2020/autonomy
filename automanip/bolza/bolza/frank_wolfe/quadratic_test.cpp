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

#include <gtest/gtest.h>
#include <iostream>

#include "bolza/frank_wolfe/gradient_descent.hpp"

using namespace bolza;

class QuadraticCost final : public NLPCost {
 public:
  QuadraticCost() = default;
  ~QuadraticCost() = default;

  size_t SetCurrentParameter(const vector_t& x) override {
    x_ = x;
    return 0;
  }

  bool GetCost(size_t id, scalar_t& f) override {
    f = 0.5 * x_.dot(x_);
    return true;
  }

  void GetCostDerivative(size_t id, vector_t& g) override { g = x_; }

  void GetCostSecondDerivative(size_t id, matrix_t& H) override { H.setIdentity(x_.size(), x_.size()); }

  void ClearCache() override {}

 private:
  vector_t x_;
};

class QuadraticConstraints final : public NLPConstraints {
 public:
  QuadraticConstraints(vector_t minX, vector_t maxX) {
    size_t numParameters = maxX.size();

    Cm_.resize(2 * numParameters, numParameters);
    Dv_.resize(2 * numParameters);

    Cm_.topRows(numParameters) = -Eigen::MatrixXd::Identity(numParameters, numParameters);
    Cm_.bottomRows(numParameters) = Eigen::MatrixXd::Identity(numParameters, numParameters);

    Dv_.head(numParameters) = maxX;
    Dv_.tail(numParameters) = -minX;
  }

  ~QuadraticConstraints() = default;

  void SetCurrentParameter(const vector_t& x) override { x_ = x; }

  void GetLinearInequalityConstraint(vector_t& h) override { h = Cm_ * x_ + Dv_; }

  void GetLinearInequalityConstraintDerivative(matrix_t& dhdx) override { dhdx = Cm_; }

 private:
  vector_t x_;
  matrix_t Cm_;
  vector_t Dv_;
};

TEST(QuadraticTest, QuadraticTest) {
  NLPSettings nlpSettings;
  nlpSettings.displayInfo_ = true;
  nlpSettings.maxIterations_ = 500;
  nlpSettings.minRelCost_ = 1e-6;
  nlpSettings.maxLearningRate_ = 1.0;
  nlpSettings.minLearningRate_ = 1e-4;
  nlpSettings.useAscendingLineSearchNLP_ = false;

  GradientDescent nlpSolver(nlpSettings);
  std::unique_ptr<QuadraticCost> costPtr(new QuadraticCost);

  vector_t maxX = Eigen::Vector2d(3.0, 3.0);
  vector_t minX = Eigen::Vector2d(1.0, 1.0);
  std::unique_ptr<QuadraticConstraints> constraintsPtr(new QuadraticConstraints(minX, maxX));

  Eigen::Vector2d initParameters = 0.5 * (maxX + minX) + 0.5 * (maxX - minX).cwiseProduct(Eigen::Vector2d::Random());
  nlpSolver.Run(initParameters, 0.1 * Eigen::Vector2d::Ones(), costPtr.get(), constraintsPtr.get());

  double cost;
  nlpSolver.GetCost(cost);
  vector_t parameters;
  nlpSolver.GetParameters(parameters);

  std::cout << "cost: " << Cost << std::endl;
  std::cout << "parameters: " << parameters.transpose() << std::endl;

  const double optimalCost = 1.0;
  const Eigen::Vector2d optimalParameters = Eigen::Vector2d::Ones();

  ASSERT_NEAR(cost, optimalCost, nlpSettings.minRelCost_) << "MESSAGE: Frank_Wolfe failed in the Quadratic test!";
}
