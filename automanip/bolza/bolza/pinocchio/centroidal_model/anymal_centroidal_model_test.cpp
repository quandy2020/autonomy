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

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include "bolza/pinocchio/centroidal/centroidal_model_rbd_conversions.hpp"
#include "bolza/pinocchio/centroidal/factory_functions.hpp"
#include "bolza/pinocchio/centroidal/model_helper_functions.hpp"
#include "bolza/pinocchio/centroidal/pinocchio_centroidal_dynamics.hpp"
#include "bolza/pinocchio/centroidal/pinocchio_centroidal_dynamics_ad.hpp"

#include "helpers/definitions.hpp"

using namespace bolza;
using namespace centroidal_model;

class TestAnymalCentroidalModel : public ::testing::TestWithParam<CentroidalModelType> {
 public:
  using Matrix6x = Eigen::Matrix<scalar_t, 6, Eigen::Dynamic>;
  TestAnymalCentroidalModel() {
    srand(0);
    pinocchioInterfacePtr.reset(new PinocchioInterface(CreatePinocchioInterface(anymalUrdfFile)));
  }

  CentroidalModelInfo createInfo(CentroidalModelType type) const {
    const size_t nq = pinocchioInterfacePtr->GetModel().nq;
    const size_t numJoints = nq - 6;
    return CreateCentroidalModelInfo(*pinocchioInterfacePtr, type, GetInitialState().tail(numJoints), anymal3DofContactNames,
                                     anymal6DofContactNames);
  }

  std::unique_ptr<CentroidalModelPinocchioMapping> createMapping(CentroidalModelType type) const {
    std::unique_ptr<CentroidalModelPinocchioMapping> mappingPtr(new CentroidalModelPinocchioMapping(createInfo(type)));
    mappingPtr->SetPinocchioInterface(*pinocchioInterfacePtr);
    return mappingPtr;
  }

  static constexpr scalar_t tol = 1e-9;
  static constexpr size_t numTests = 100;
  std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr;
};

constexpr scalar_t TestAnymalCentroidalModel::Tol;
constexpr size_t TestAnymalCentroidalModel::numTests;

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TEST_P(TestAnymalCentroidalModel, info) {
  const CentroidalModelType type = GetParam();
  const auto info = createInfo(type);
  EXPECT_EQ(info.stateDim, anymal::STATE_DIM);
  EXPECT_EQ(info.inputDim, anymal::INPUT_DIM);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TEST_P(TestAnymalCentroidalModel, dynamis_flowMap) {
  const CentroidalModelType type = GetParam();
  auto mappingPtr = createMapping(type);
  const auto& info = mappingPtr->GetCentroidalModelInfo();

  // Analytical model
  PinocchioCentroidalDynamics anymalDynamics(createInfo(type));
  anymalDynamics.SetPinocchioInterface(*pinocchioInterfacePtr);

  // CppAD model
  const std::string modelName = "TestAnymal" + ToString(type) + "Ad";
  PinocchioCentroidalDynamicsAD anymalDynamicsAd(*pinocchioInterfacePtr, createInfo(type), modelName);

  for (size_t i = 0; i < numTests; i++) {
    const scalar_t time = 0.0;
    const vector_t state = 10.0 * vector_t::Random(anymal::STATE_DIM);
    const vector_t input = 10000.0 * vector_t::Random(anymal::INPUT_DIM);

    const vector_t qPinocchio = mappingPtr->GetPinocchioJointPosition(state);
    UpdateCentroidalDynamics(*pinocchioInterfacePtr, info, qPinocchio);

    const auto stateDerivative = anymalDynamics.GetValue(time, state, input);
    const auto stateDerivativeAd = anymalDynamicsAd.GetValue(time, state, input);

    const vector_t vPinocchio = mappingPtr->GetPinocchioJointVelocity(state, input);
    UpdateCentroidalDynamicsDerivatives(*pinocchioInterfacePtr, info, qPinocchio, vPinocchio);

    const auto linearApproximation = anymalDynamics.GetLinearApproximation(time, state, input);
    const auto linearApproximationAd = anymalDynamicsAd.GetLinearApproximation(time, state, input);

    EXPECT_TRUE(stateDerivative.isApprox(stateDerivativeAd, tol));
    EXPECT_TRUE(stateDerivative.isApprox(linearApproximation.f, tol));
    EXPECT_TRUE(linearApproximationAd.f.isApprox(linearApproximation.f, tol));
    EXPECT_TRUE(linearApproximationAd.dfdx.isApprox(linearApproximation.dfdx, tol));
    EXPECT_TRUE(linearApproximationAd.dfdu.isApprox(linearApproximation.dfdu, tol));
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TEST_P(TestAnymalCentroidalModel, rbd_conversion) {
  const CentroidalModelType type = GetParam();
  const auto info = createInfo(type);

  // Analytical model
  CentroidalModelRbdConversions rbdConversions(*pinocchioInterfacePtr, info);

  // bolza --> rbd --> bolza
  for (size_t i = 0; i < numTests; i++) {
    const vector_t state = 10.0 * vector_t::Random(anymal::STATE_DIM);
    const vector_t input = 10000.0 * vector_t::Random(anymal::INPUT_DIM);

    const vector_t rbdState = rbdConversions.ComputeRbdStateFromCentroidalModel(state, input);
    const vector_t reconstructedState = rbdConversions.ComputeCentroidalStateFromRbdModel(rbdState);

    EXPECT_TRUE(state.isApprox(reconstructedState, tol));
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
INSTANTIATE_TEST_CASE_P(TestAnymalCentroidalModelWithParam, TestAnymalCentroidalModel,
                        testing::ValuesIn({CentroidalModelType::FullCentroidalDynamics, CentroidalModelType::SingleRigidBodyDynamics}),
                        [](const testing::TestParamInfo<TestAnymalCentroidalModel::ParamType>& info) { return ToString(info.param); });
