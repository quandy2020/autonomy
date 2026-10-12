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
#include <filesystem>
#include <functional>
#include <iostream>

#include "LinearSystemDynamicsAD.h"
#include "bolza/core/dynamics/linear_system_dynamics.hpp"
#include "helpers/tools_test.hpp"

using namespace bolza;

class testCppADCG_dynamicsFixture : public ::testing::Test {
 public:
  const size_t stateDim_ = 4;
  const size_t inputDim_ = 2;

  testCppADCG_dynamicsFixture() { Create(); };

  void Create() {
    // Define dynamics parameters
    matrix_t A = matrix_t::Random(stateDim_, stateDim_);
    matrix_t B = matrix_t::Random(stateDim_, inputDim_);
    matrix_t G = matrix_t::Random(stateDim_, stateDim_);

    linearSystem_.Reset(new LinearSystemDynamics(A, B, G));

    std::filesystem::path filePath(__FILE__);
    std::string libraryFolder = filePath.parent_path().generic_string() + "/testCppADCG_generated";
    adLinearSystem_.Reset(new LinearSystemDynamicsAD(A, B, G));

    adLinearSystem_->Initialize(stateDim_, inputDim_, "testCppADCG_dynamics", libraryFolder, true, true);
  }

  std::unique_ptr<LinearSystemDynamics> linearSystem_;
  std::unique_ptr<LinearSystemDynamicsAD> adLinearSystem_;
};

/******************************************************************************/
/******************************************************************************/
/***************************************************************************** */
void checkSystemDynamics(const size_t numTests, SystemDynamicsBase* const linearSystem1, SystemDynamicsBase* const linearSystem2,
                         bool& success, size_t stateDim_, size_t inputDim_) {
  vector_t x;
  vector_t u;
  const scalar_t t = 0;
  const scalar_t precision = 1e-9;

  success = true;
  for (size_t it = 0; it < numTests && success; it++) {
    x.setRandom(stateDim_);
    u.setRandom(inputDim_);

    VectorFunctionLinearApproximation f1 = linearSystem1->LinearApproximation(t, x, u);
    VectorFunctionLinearApproximation f2 = linearSystem2->LinearApproximation(t, x, u);

    if (!IsApprox(f1, f2, precision)) {
      std::cout << "systemDynamic1:\n" << f1 << std::endl;
      std::cout << "systemDynamic2:\n" << f2 << std::endl;
      success = false;
    }

    VectorFunctionLinearApproximation g1 = linearSystem1->JumpMapLinearApproximation(t, x);
    VectorFunctionLinearApproximation g2 = linearSystem2->JumpMapLinearApproximation(t, x);
    if (!IsApprox(g1, g2, precision)) {
      std::cout << "jumpMap1:\n" << g1.dfdx << std::endl;
      std::cout << "jumpMap2:\n" << g2.dfdx << std::endl;
      success = false;
    }
  }
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
TEST_F(testCppADCG_dynamicsFixture, system_dynamics_test) {
  bool success;
  checkSystemDynamics(100, linearSystem_.get(), adLinearSystem_.get(), success, stateDim_, inputDim_);

  ASSERT_TRUE(success);
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
TEST_F(testCppADCG_dynamicsFixture, clone_test) {
  std::unique_ptr<SystemDynamicsBase> adLinearSystemPtr(adLinearSystem_->Clone());
  bool success;
  checkSystemDynamics(100, linearSystem_.get(), adLinearSystemPtr.get(), success, stateDim_, inputDim_);

  ASSERT_TRUE(success);
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
TEST_F(testCppADCG_dynamicsFixture, multithread_test) {
  std::unique_ptr<SystemDynamicsBase> linearSystemPtr(linearSystem_->Clone());
  std::unique_ptr<SystemDynamicsBase> adLinearSystemPtr(adLinearSystem_->Clone());

  bool success = false;
  std::thread thread1(checkSystemDynamics, 10000, linearSystem_.get(), adLinearSystem_.get(), std::ref(success), stateDim_, inputDim_);

  bool successClone = false;
  std::thread thread2(checkSystemDynamics, 10000, linearSystemPtr.get(), adLinearSystemPtr.get(), std::ref(successClone), stateDim_,
                      inputDim_);

  if (thread1.joinable()) {
    thread1.join();
  }
  if (thread2.joinable()) {
    thread2.join();
  }

  ASSERT_TRUE(success && successClone);
}
