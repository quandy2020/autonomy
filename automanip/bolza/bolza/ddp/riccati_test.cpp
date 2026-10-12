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

#include <memory>

#include <gtest/gtest.h>

#include <bolza/core/misc/linear_algebra.hpp>
#include <bolza/core/misc/random_matrices.hpp>
#include <bolza/ddp/riccati_equations/continuous_time_riccati_equations.hpp>

class RiccatiInitializer {
 public:
  using riccati_t = bolza::ContinuousTimeRiccatiEquations;

  bolza::scalar_array_t timeStamp;
  std::vector<bolza::ModelData> projectedModelDataTrajectory;

  bolza::size_array_t eventsPastTheEndIndeces;
  std::vector<bolza::ModelData> modelDataEventTimesArray;

  std::vector<bolza::riccati_modification::Data> riccatiModificationTrajectory;

  RiccatiInitializer(const int stateDim, const int inputDim) {
    timeStamp = bolza::scalar_array_t{0.0, 1.0};

    bolza::ModelData projectedModelData;
    projectedModelData.stateDim = stateDim;
    projectedModelData.inputDim = inputDim;
    projectedModelData.dynamicsBias = bolza::vector_t::Random(stateDim);
    projectedModelData.dynamics.dfdx = bolza::matrix_t::Random(stateDim, stateDim);
    projectedModelData.dynamics.dfdu = bolza::matrix_t::Random(stateDim, inputDim);
    projectedModelData.cost.f = bolza::vector_t::Random(1)(0);
    projectedModelData.cost.dfdx = bolza::vector_t::Random(stateDim);
    projectedModelData.cost.dfdxx = bolza::LinearAlgebra::GenerateSPDmatrix<bolza::matrix_t>(stateDim);
    projectedModelData.cost.dfdu = bolza::vector_t::Random(inputDim);
    projectedModelData.cost.dfduu.setIdentity(inputDim,
                                              inputDim);  // Important: It is identity since it is a projected projectedModelData!
    projectedModelData.cost.dfdux = bolza::matrix_t::Random(inputDim, stateDim);
    projectedModelData.stateEqConstraint.setZero(0, stateDim);
    projectedModelData.stateInputEqConstraint.setZero(inputDim, stateDim, inputDim);

    projectedModelDataTrajectory = std::vector<bolza::ModelData>{projectedModelData, projectedModelData};

    bolza::riccati_modification::Data riccatiModification;
    riccatiModification.deltaQm_ = 0.1 * bolza::LinearAlgebra::GenerateSPDmatrix<bolza::matrix_t>(stateDim);
    riccatiModification.deltaGv_ = bolza::vector_t::Zero(inputDim);
    riccatiModification.deltaGm_ = bolza::matrix_t::Zero(inputDim, stateDim);
    riccatiModification.constraintRangeProjector_.setZero(inputDim, 0);
    bolza::LinearAlgebra::ComputeInverseMatrixUUT(projectedModelData.cost.dfduu, riccatiModification.constraintNullProjector_);

    riccatiModificationTrajectory = std::vector<bolza::riccati_modification::Data>{riccatiModification, riccatiModification};
  }

  void Initialize(riccati_t& riccati) {
    riccati.SetData(&timeStamp, &projectedModelDataTrajectory, &eventsPastTheEndIndeces, &modelDataEventTimesArray,
                    &riccatiModificationTrajectory);
  }
};

TEST(RiccatiTest, compareImplementations) {
  constexpr int STATE_DIM = 48;
  constexpr int INPUT_DIM = 10;

  using riccati_t = bolza::ContinuousTimeRiccatiEquations;

  riccati_t riccatiEquationPrecompute(true);
  riccati_t riccatiEquationNoPrecompute(false);

  RiccatiInitializer ri(STATE_DIM, INPUT_DIM);
  ri.Initialize(riccatiEquationPrecompute);
  ri.Initialize(riccatiEquationNoPrecompute);

  bolza::vector_t S = bolza::vector_t::Random(bolza::s_vector_dim(STATE_DIM));
  bolza::vector_t dSdz_precompute = riccatiEquationPrecompute.ComputeFlowMap(0.6, S);
  bolza::vector_t dSdz_noPrecompute = riccatiEquationNoPrecompute.ComputeFlowMap(0.6, S);

  EXPECT_LE((dSdz_precompute - dSdz_noPrecompute).array().abs().maxCoeff(), 1e-9);
}

TEST(RiccatiTest, testFlattenSMatrix) {
  const int stateDim = 4;
  using riccati_t = bolza::ContinuousTimeRiccatiEquations;

  bolza::vector_t allSs, allSs_expect;
  bolza::matrix_t Sm;
  bolza::vector_t Sv;
  bolza::scalar_t s;

  Sm.resize(stateDim, stateDim);
  Sm << 1, 2, 4, 7,  // clang-format off
        2, 3, 5, 8,
        4, 5, 6, 9,
        7, 8, 9, 10;  // clang-format on
  Sv.resize(stateDim);
  Sv << 11, 12, 13, 14;
  s = 15;

  allSs_expect.resize(15);
  allSs_expect << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15;

  allSs = riccati_t::Convert2Vector(Sm, Sv, s);

  EXPECT_EQ(allSs, allSs_expect);
}

TEST(RiccatiTest, testFlattenAndUnflatten) {
  const int stateDim = 42;
  using riccati_t = bolza::ContinuousTimeRiccatiEquations;

  bolza::vector_t allSs;
  bolza::matrix_t Sm, Sm_out;
  bolza::vector_t Sv, Sv_out;
  bolza::scalar_t s, s_out;

  Sm.setRandom(stateDim, stateDim);
  Sm = (Sm + Sm.transpose()).eval();
  Sv.setRandom(stateDim);
  s = bolza::vector_t::Random(1)(0);

  Sm_out.setZero(stateDim, stateDim);
  Sv_out.setZero(stateDim);
  s_out = 0.0;

  allSs = riccati_t::Convert2Vector(Sm, Sv, s);
  riccati_t::Convert2Matrix(allSs, Sm_out, Sv_out, s_out);

  EXPECT_EQ(s, s_out);
  ASSERT_TRUE(Sv.isApprox(Sv_out));
  ASSERT_TRUE(Sm.isApprox(Sm_out));
}
