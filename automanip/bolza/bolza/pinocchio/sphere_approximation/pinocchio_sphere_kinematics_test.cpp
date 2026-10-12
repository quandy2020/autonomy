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

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/geometry.hpp>

#include <bolza/pinocchio/interface/urdf.hpp>
#include <bolza/pinocchio/sphere/pinocchio_sphere_kinematics.hpp>
#include <bolza/pinocchio/sphere/pinocchio_sphere_kinematics_cpp_ad.hpp>

#include <robotic_assets/package_path.hpp>
#include <bolza/tools/common/skew_symmetric_matrix.hpp>

#include <bolza/pinocchio/interface/urdf.hpp>

#include <gtest/gtest.h>

using vector3_t = Eigen::Matrix<bolza::scalar_t, 3, 1>;

template <typename SCALAR>
class DummyMapping final : public bolza::PinocchioStateInputMapping<SCALAR> {
 public:
  using scalar_t = SCALAR;
  using vector_t = Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>;
  using matrix_t = Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic>;

  DummyMapping() = default;
  ~DummyMapping() override = default;
  DummyMapping<SCALAR>* Clone() const override { return new DummyMapping<SCALAR>(*this); }

  vector_t GetPinocchioJointPosition(const vector_t& state) const override { return state; }

  vector_t GetPinocchioJointVelocity(const vector_t& state, const vector_t& input) const override { return input; }

  std::pair<matrix_t, matrix_t> GetOcs2Jacobian(const vector_t& state, const matrix_t& Jq, const matrix_t& Jv) const override {
    return {Jq, Jv};
  }
};

class TestSphereKinematics : public ::testing::Test {
 public:
  using quaternion_t = Eigen::Quaternion<bolza::scalar_t>;

  TestSphereKinematics() {
    const std::string urdfFile = bolza::robotic_assets::GetPath() + "/resources/mobile_manipulator/mabi_mobile/urdf/mabi_mobile.urdf";
    pinocchioInterfacePtr.Reset(new bolza::PinocchioInterface(bolza::GetPinocchioInterfaceFromUrdfFile(urdfFile)));
    pinocchioSphereInterfacePtr.Reset(new bolza::PinocchioSphereInterface(*pinocchioInterfacePtr, {"ARM", "SHOULDER", "FOREARM", "WRIST_1"},
                                                                         {0.20, 0.10, 0.05, 0.05}, 0.7));
    sphereKinematicsPtr.Reset(new bolza::PinocchioSphereKinematics(*pinocchioSphereInterfacePtr, pinocchioMapping));
    sphereKinematicsCppAdPtr.Reset(new bolza::PinocchioSphereKinematicsCppAd(
        *pinocchioInterfacePtr, *pinocchioSphereInterfacePtr, pinocchioMappingCppAd, pinocchioInterfacePtr->GetModel().njoints, 0,
        "pinocchio_sphere_kinematics", "/tmp/bolza", true, true));

    x.resize(pinocchioInterfacePtr->GetModel().njoints);
    // taken form config/mpc/task.info
    x(0) = 2.5;   // SH_ROT
    x(1) = -1.0;  // SH_FLE
    x(2) = 1.5;   // EL_FLE
    x(3) = 0.0;   // EL_ROT
    x(4) = 1.0;   // WR_FLE
    x(5) = 0.0;   // WR_ROT

    q = pinocchioMapping.GetPinocchioJointPosition(x);
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
  }

  bolza::vector_t x;  // state
  bolza::vector_t q;  // pinocchio joint positions

  std::unique_ptr<bolza::PinocchioInterface> pinocchioInterfacePtr;
  std::unique_ptr<bolza::PinocchioSphereInterface> pinocchioSphereInterfacePtr;
  std::unique_ptr<bolza::PinocchioSphereKinematics> sphereKinematicsPtr;
  std::unique_ptr<bolza::PinocchioSphereKinematicsCppAd> sphereKinematicsCppAdPtr;
  DummyMapping<bolza::scalar_t> pinocchioMapping;
  DummyMapping<bolza::ad_scalar_t> pinocchioMappingCppAd;
};

TEST_F(TestSphereKinematics, testKinematicsPosition) {
  const auto& model = pinocchioInterfacePtr->GetModel();
  auto& data = pinocchioInterfacePtr->GetData();
  const auto& geometryModel = pinocchioSphereInterfacePtr->GetGeometryModel();
  const auto geomObjIds = pinocchioSphereInterfacePtr->GetGeomObjIds();

  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::computeJointJacobians(model, data, q);

  const auto parentJointId = geometryModel.geometryObjects[geomObjIds[0]].parentJoint;
  const auto& placement = geometryModel.geometryObjects[geomObjIds[0]].placement;
  const vector3_t pos =
      data.oMi[parentJointId].rotation() *
          (placement.rotation() * pinocchioSphereInterfacePtr->GetSphereCentersToObjectCenter(0)[0] + placement.translation()) +
      data.oMi[parentJointId].translation();

  const vector3_t sphereOffset = pos - data.oMi[parentJointId].translation();
  bolza::matrix_t jointJacobian = bolza::matrix_t::Zero(6, model.nv);
  pinocchio::getJointJacobian(model, data, parentJointId, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jointJacobian);
  const bolza::matrix_t sphereJq = jointJacobian.topRows<3>() - bolza::SkewSymmetricMatrix(sphereOffset) * jointJacobian.bottomRows<3>();
  bolza::matrix_t sphereJx;
  std::tie(sphereJx, std::ignore) = pinocchioMapping.GetOcs2Jacobian(x, sphereJq.topRows<3>(), bolza::matrix_t::Zero(0, model.nq));

  sphereKinematicsPtr->SetPinocchioInterface(*pinocchioInterfacePtr);
  const vector3_t spherePos = sphereKinematicsPtr->GetPosition(x)[0];
  const bolza::VectorFunctionLinearApproximation spherePosLin = sphereKinematicsPtr->GetPositionLinearApproximation(x)[0];

  EXPECT_TRUE(spherePos.isApprox(spherePos));
  EXPECT_TRUE(spherePos.isApprox(spherePosLin.f));
  EXPECT_TRUE(sphereJx.isApprox(spherePosLin.dfdx));
}

TEST_F(TestSphereKinematics, testPosition) {
  const auto& model = pinocchioInterfacePtr->GetModel();
  auto& data = pinocchioInterfacePtr->GetData();

  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  sphereKinematicsPtr->SetPinocchioInterface(*pinocchioInterfacePtr);

  const auto spherePos = sphereKinematicsPtr->GetPosition(x)[0];
  const auto spherePosAd = sphereKinematicsCppAdPtr->GetPosition(x)[0];
  EXPECT_TRUE(spherePos.isApprox(spherePosAd));
}

TEST_F(TestSphereKinematics, testPositionApproximation) {
  const auto& model = pinocchioInterfacePtr->GetModel();
  auto& data = pinocchioInterfacePtr->GetData();

  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::computeJointJacobians(model, data);

  sphereKinematicsPtr->SetPinocchioInterface(*pinocchioInterfacePtr);

  const auto spherePosLin = sphereKinematicsPtr->GetPositionLinearApproximation(x)[0];
  const auto spherePosLinAd = sphereKinematicsCppAdPtr->GetPositionLinearApproximation(x)[0];
  compareApproximation(spherePosLin, spherePosLinAd);
}

TEST_F(TestSphereKinematics, testClone) {
  const auto& model = pinocchioInterfacePtr->GetModel();
  auto& data = pinocchioInterfacePtr->GetData();

  auto clonePtr = std::unique_ptr<bolza::PinocchioSphereKinematics>(sphereKinematicsPtr->Clone());
  auto cloneCppAdPtr = std::unique_ptr<bolza::PinocchioSphereKinematicsCppAd>(sphereKinematicsCppAdPtr->Clone());

  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::computeJointJacobians(model, data);

  clonePtr->SetPinocchioInterface(*pinocchioInterfacePtr);

  const auto spherePos = clonePtr->GetPosition(x)[0];
  const auto spherePosAd = cloneCppAdPtr->GetPosition(x)[0];
  EXPECT_TRUE(spherePos.isApprox(spherePosAd));
}