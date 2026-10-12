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

#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames-derivatives.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics-derivatives.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <bolza/pinocchio/interface/pinocchio_end_effector_kinematics.hpp>
#include <bolza/pinocchio/interface/pinocchio_end_effector_kinematics_cpp_ad.hpp>
#include <bolza/pinocchio/interface/urdf.hpp>

#include <bolza/core/automatic_differentiation/finite_difference_methods.hpp>
#include <bolza/tools/common/angular_velocity_mapping.hpp>
#include <bolza/tools/common/rotation_transforms.hpp>

#include <gtest/gtest.h>

#include "ManipulatorArmUrdf.h"

template <typename SCALAR>
class ManipulatorMapping final : public bolza::PinocchioStateInputMapping<SCALAR> {
 public:
  using scalar_t = SCALAR;
  using vector_t = Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>;
  using matrix_t = Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic>;

  ManipulatorMapping() = default;
  ~ManipulatorMapping() override = default;
  ManipulatorMapping<SCALAR>* Clone() const override { return new ManipulatorMapping<SCALAR>(*this); }

  vector_t GetPinocchioJointPosition(const vector_t& state) const override { return state; }

  vector_t GetPinocchioJointVelocity(const vector_t& state, const vector_t& input) const override { return input; }

  std::pair<matrix_t, matrix_t> GetOcs2Jacobian(const vector_t& state, const matrix_t& Jq, const matrix_t& Jv) const override {
    return {Jq, Jv};
  }
};

class TestEndEffectorKinematics : public ::testing::Test {
 public:
  using quaternion_t = Eigen::Quaternion<bolza::scalar_t>;

  TestEndEffectorKinematics() {
    auto pinocchioInterface = bolza::GetPinocchioInterfaceFromUrdfString(manipulatorArmUrdf);

    pinocchioInterfacePtr.reset(new bolza::PinocchioInterface(pinocchioInterface));
    eeKinematicsPtr.reset(new bolza::PinocchioEndEffectorKinematics(pinocchioInterface, pinocchioMapping, {"WRIST_2"}));
    eeKinematicsCppAdPtr.reset(new bolza::PinocchioEndEffectorKinematicsCppAd(
        pinocchioInterface, pinocchioMappingCppAd, {"WRIST_2"}, 6, 6, "pinocchio_end_effector_kinematics", "/tmp/bolza", true, false));

    x.resize(6);
    x << 2.5, -1.0, 1.5, 0.0, 1.0, 0.0;
    u.setOnes(6);

    q = pinocchioMapping.GetPinocchioJointPosition(x);
    v = pinocchioMapping.GetPinocchioJointVelocity(x, u);
  }

  void compareApproximation(const bolza::VectorFunctionLinearApproximation& f1, const bolza::VectorFunctionLinearApproximation& f2,
                            bool functionOfInput = false) {
    if (!f1.f.isApprox(f2.f)) {
      std::cerr << "f1.f  " << f1.f.transpose() << '\n';
      std::cerr << "f2.f  " << f2.f.transpose() << '\n';
    }

    if (!f1.dfdx.isApprox(f2.dfdx)) {
      std::cerr << "f1.dfdx\n" << f1.dfdx << '\n';
      std::cerr << "f2.dfdx\n" << f2.dfdx << '\n';
    }

    if (functionOfInput && !f1.dfdu.isApprox(f2.dfdu)) {
      std::cerr << "f1.dfdu\n" << f1.dfdu << '\n';
      std::cerr << "f2.dfdu\n" << f2.dfdu << '\n';
    }

    EXPECT_TRUE(f1.f.isApprox(f2.f));
    EXPECT_TRUE(f1.dfdx.isApprox(f2.dfdx));
    if (functionOfInput) {
      EXPECT_TRUE(f1.dfdu.isApprox(f2.dfdu));
    }
  }

  bolza::vector_t x;  // state
  bolza::vector_t u;  // input
  bolza::vector_t q;  // pinocchio joint positions
  bolza::vector_t v;  // pinocchio joint velocities

  std::unique_ptr<bolza::PinocchioInterface> pinocchioInterfacePtr;
  std::unique_ptr<bolza::PinocchioEndEffectorKinematics> eeKinematicsPtr;
  std::unique_ptr<bolza::PinocchioEndEffectorKinematicsCppAd> eeKinematicsCppAdPtr;
  ManipulatorMapping<bolza::scalar_t> pinocchioMapping;
  ManipulatorMapping<bolza::ad_scalar_t> pinocchioMappingCppAd;
};

TEST_F(TestEndEffectorKinematics, testKinematicsPosition) {
  const auto& model = pinocchioInterfacePtr->GetModel();
  auto& data = pinocchioInterfacePtr->GetData();

  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::computeJointJacobians(model, data);

  const auto id = model.getBodyId("WRIST_2");
  const bolza::vector_t pos = data.oMf[id].translation();
  bolza::matrix_t J = bolza::matrix_t::Zero(6, model.nq);
  pinocchio::getFrameJacobian(model, data, id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J);

  eeKinematicsPtr->SetPinocchioInterface(*pinocchioInterfacePtr);
  const auto eePos = eeKinematicsPtr->GetPosition(x)[0];
  const auto eePosLin = eeKinematicsPtr->GetPositionLinearApproximation(x)[0];

  EXPECT_TRUE(pos.isApprox(eePos));
  EXPECT_TRUE(pos.isApprox(eePosLin.f));
  EXPECT_TRUE(J.topRows<3>().isApprox(eePosLin.dfdx));
}

TEST_F(TestEndEffectorKinematics, testPosition) {
  const auto& model = pinocchioInterfacePtr->GetModel();
  auto& data = pinocchioInterfacePtr->GetData();

  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  eeKinematicsPtr->SetPinocchioInterface(*pinocchioInterfacePtr);

  const auto eePos = eeKinematicsPtr->GetPosition(x)[0];
  const auto eePosAd = eeKinematicsCppAdPtr->GetPosition(x)[0];
  EXPECT_TRUE(eePos.isApprox(eePosAd));
}

TEST_F(TestEndEffectorKinematics, testPositionApproximation) {
  const auto& model = pinocchioInterfacePtr->GetModel();
  auto& data = pinocchioInterfacePtr->GetData();

  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::computeJointJacobians(model, data);

  eeKinematicsPtr->SetPinocchioInterface(*pinocchioInterfacePtr);

  const auto eePosLin = eeKinematicsPtr->GetPositionLinearApproximation(x)[0];
  const auto eePosLinAd = eeKinematicsCppAdPtr->GetPositionLinearApproximation(x)[0];
  compareApproximation(eePosLin, eePosLinAd);
}

TEST_F(TestEndEffectorKinematics, testVelocity) {
  const auto& model = pinocchioInterfacePtr->GetModel();
  auto& data = pinocchioInterfacePtr->GetData();

  const auto a = bolza::vector_t::Zero(q.rows());
  pinocchio::computeForwardKinematicsDerivatives(model, data, q, v, a);

  eeKinematicsPtr->SetPinocchioInterface(*pinocchioInterfacePtr);

  const auto eeVel = eeKinematicsPtr->GetVelocity(x, u)[0];
  const auto eeVelAd = eeKinematicsCppAdPtr->GetVelocity(x, u)[0];
  EXPECT_TRUE(eeVel.isApprox(eeVelAd));
}

TEST_F(TestEndEffectorKinematics, testVelocityApproximation) {
  const auto& model = pinocchioInterfacePtr->GetModel();
  auto& data = pinocchioInterfacePtr->GetData();

  const auto a = bolza::vector_t::Zero(q.rows());
  pinocchio::computeForwardKinematicsDerivatives(model, data, q, v, a);

  eeKinematicsPtr->SetPinocchioInterface(*pinocchioInterfacePtr);

  const auto eeVelLin = eeKinematicsPtr->GetVelocityLinearApproximation(x, u)[0];
  const auto eeVelLinAd = eeKinematicsCppAdPtr->GetVelocityLinearApproximation(x, u)[0];
  compareApproximation(eeVelLin, eeVelLinAd, /* functionOfInput = */ true);
}

TEST_F(TestEndEffectorKinematics, testOrientationError) {
  const auto& model = pinocchioInterfacePtr->GetModel();
  auto& data = pinocchioInterfacePtr->GetData();
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  eeKinematicsPtr->SetPinocchioInterface(*pinocchioInterfacePtr);

  const quaternion_t qRef(1, 0, 0, 0);
  const auto eeOrientationError = eeKinematicsPtr->GetOrientationError(x, {qRef})[0];
  const auto eeOrientationErrorAd = eeKinematicsCppAdPtr->GetOrientationError(x, {qRef})[0];
  EXPECT_TRUE(eeOrientationError.isApprox(eeOrientationErrorAd));
}

TEST_F(TestEndEffectorKinematics, testOrientationErrorApproximation) {
  const auto& model = pinocchioInterfacePtr->GetModel();
  auto& data = pinocchioInterfacePtr->GetData();
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::computeJointJacobians(model, data);
  eeKinematicsPtr->SetPinocchioInterface(*pinocchioInterfacePtr);

  const quaternion_t qRef(1, 0, 0, 0);
  const auto eeOrientationErrorLin = eeKinematicsPtr->GetOrientationErrorLinearApproximation(x, {qRef})[0];
  const auto eeOrientationErrorLinAd = eeKinematicsCppAdPtr->GetOrientationErrorLinearApproximation(x, {qRef})[0];
  compareApproximation(eeOrientationErrorLin, eeOrientationErrorLinAd);
}

TEST_F(TestEndEffectorKinematics, testOrientationError_zeroError) {
  const auto& model = pinocchioInterfacePtr->GetModel();
  auto& data = pinocchioInterfacePtr->GetData();
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  const auto frameId = model.getBodyId("WRIST_2");
  const quaternion_t qRef = bolza::MatrixToQuaternion(data.oMf[frameId].rotation());
  eeKinematicsPtr->SetPinocchioInterface(*pinocchioInterfacePtr);

  const auto eeOrientationError = eeKinematicsPtr->GetOrientationError(x, {qRef})[0];
  const auto eeOrientationErrorAd = eeKinematicsCppAdPtr->GetOrientationError(x, {qRef})[0];

  EXPECT_TRUE(eeOrientationError.norm() < 1e-9);
  EXPECT_TRUE(eeOrientationErrorAd.norm() < 1e-9);
}

TEST_F(TestEndEffectorKinematics, testClone) {
  const auto& model = pinocchioInterfacePtr->GetModel();
  auto& data = pinocchioInterfacePtr->GetData();

  auto clonePtr = std::unique_ptr<bolza::PinocchioEndEffectorKinematics>(eeKinematicsPtr->Clone());
  auto cloneCppAdPtr = std::unique_ptr<bolza::PinocchioEndEffectorKinematicsCppAd>(eeKinematicsCppAdPtr->Clone());

  const auto a = bolza::vector_t::Zero(q.rows());
  pinocchio::computeForwardKinematicsDerivatives(model, data, q, v, a);
  pinocchio::updateFramePlacements(model, data);

  clonePtr->SetPinocchioInterface(*pinocchioInterfacePtr);

  const auto eePos = clonePtr->GetPosition(x)[0];
  const auto eePosAd = cloneCppAdPtr->GetPosition(x)[0];
  EXPECT_TRUE(eePos.isApprox(eePosAd));

  const auto eeVel = clonePtr->GetVelocity(x, u)[0];
  const auto eeVelAd = cloneCppAdPtr->GetVelocity(x, u)[0];
  EXPECT_TRUE(eeVel.isApprox(eeVelAd));
}

/* Test to understand the frame jacobian */
TEST_F(TestEndEffectorKinematics, testPinocchioOrientationErrorJacoiban) {
  const auto& model = pinocchioInterfacePtr->GetModel();
  auto& data = pinocchioInterfacePtr->GetData();
  const auto v = bolza::vector_t::Zero(model.nv);
  const auto frameId = model.getBodyId("WRIST_2");
  const quaternion_t qRef(1, 0, 0, 0);
  const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;
  const bolza::scalar_t eps = 1e-9;
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::computeJointJacobians(model, data);

  // Check geometric jacobian
  bolza::matrix_t J = bolza::matrix_t::Zero(6, model.nq);
  pinocchio::getFrameJacobian(model, data, frameId, rf, J);

  auto func1 = [&](const bolza::vector_t& v) -> bolza::vector_t {
    pinocchio::forwardKinematics(model, data, x, v);
    pinocchio::updateFramePlacements(model, data);
    return getFrameVelocity(model, data, frameId, rf).angular();
  };
  const bolza::matrix_t Jgeom_fd = bolza::FiniteDifferenceDerivative(func1, v, eps);
  EXPECT_TRUE(J.bottomRows<3>().isApprox(Jgeom_fd, sqrt(eps)));

  // Check analytic jacobian
  quaternion_t q = bolza::MatrixToQuaternion(data.oMf[frameId].rotation());
  bolza::matrix_t Janalytic =
      (bolza::QuaternionDistanceJacobian(q, qRef) * bolza::AngularVelocityToQuaternionTimeDerivative(q)) * J.bottomRows<3>();

  auto func2 = [&](const bolza::vector_t& x) -> bolza::vector_t {
    const auto q = pinocchioMapping.GetPinocchioJointPosition(x);
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
    eeKinematicsPtr->SetPinocchioInterface(*pinocchioInterfacePtr);
    return eeKinematicsPtr->GetOrientationError(x, {qRef})[0];
  };
  const bolza::matrix_t Janalytic_fd = bolza::FiniteDifferenceDerivative(func2, x, eps);
  EXPECT_TRUE(Janalytic.isApprox(Janalytic_fd, sqrt(eps)));
}
