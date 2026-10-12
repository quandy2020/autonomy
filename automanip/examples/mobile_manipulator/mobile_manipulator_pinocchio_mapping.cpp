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

#include "examples/mobile_manipulator/mobile_manipulator_pinocchio_mapping.hpp"

namespace bolza {
namespace mobile_manipulator {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
MobileManipulatorPinocchioMappingTpl<SCALAR>::MobileManipulatorPinocchioMappingTpl(ManipulatorModelInfo info)
    : modelInfo_(std::move(info)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
MobileManipulatorPinocchioMappingTpl<SCALAR>* MobileManipulatorPinocchioMappingTpl<SCALAR>::Clone() const {
  return new MobileManipulatorPinocchioMappingTpl<SCALAR>(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto MobileManipulatorPinocchioMappingTpl<SCALAR>::GetPinocchioJointPosition(const vector_t& state) const -> vector_t {
  return state;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto MobileManipulatorPinocchioMappingTpl<SCALAR>::GetPinocchioJointVelocity(const vector_t& state, const vector_t& input) const
    -> vector_t {
  vector_t vPinocchio = vector_t::Zero(modelInfo_.stateDim);
  // set velocity model based on model type
  switch (modelInfo_.manipulatorModelType) {
    case ManipulatorModelType::DefaultManipulator: {
      vPinocchio = input;
      break;
    }
    case ManipulatorModelType::FloatingArmManipulator: {
      vPinocchio.tail(modelInfo_.armDim) = input;
      break;
    }
    case ManipulatorModelType::FullyActuatedFloatingArmManipulator: {
      vPinocchio << input;
      break;
    }
    case ManipulatorModelType::WheelBasedMobileManipulator: {
      const auto theta = state(2);
      const auto v = input(0);  // forward velocity in base frame
      vPinocchio << cos(theta) * v, sin(theta) * v, input(1), input.tail(modelInfo_.armDim);
      break;
    }
    default: {
      throw std::runtime_error("The chosen manipulator model type is not supported!");
    }
  }  // end of switch-case

  return vPinocchio;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto MobileManipulatorPinocchioMappingTpl<SCALAR>::GetOcs2Jacobian(const vector_t& state, const matrix_t& Jq, const matrix_t& Jv) const
    -> std::pair<matrix_t, matrix_t> {
  // set jacobian model based on model type
  switch (modelInfo_.manipulatorModelType) {
    case ManipulatorModelType::DefaultManipulator: {
      return {Jq, Jv};
    }
    case ManipulatorModelType::FloatingArmManipulator: {
      matrix_t dfdu(Jv.rows(), modelInfo_.inputDim);
      dfdu = Jv.template rightCols(modelInfo_.armDim);
      return {Jq, dfdu};
    }
    case ManipulatorModelType::FullyActuatedFloatingArmManipulator: {
      return {Jq, Jv};
    }
    case ManipulatorModelType::WheelBasedMobileManipulator: {
      matrix_t dfdu(Jv.rows(), modelInfo_.inputDim);
      Eigen::Matrix<SCALAR, 3, 2> dvdu_base;
      const SCALAR theta = state(2);
      // clang-format off
      dvdu_base << cos(theta), SCALAR(0),
                   sin(theta), SCALAR(0),
                   SCALAR(0), SCALAR(1.0);
      // clang-format on
      dfdu.template leftCols<2>() = Jv.template leftCols<3>() * dvdu_base;
      dfdu.template rightCols(modelInfo_.armDim) = Jv.template rightCols(modelInfo_.armDim);
      return {Jq, dfdu};
    }
    default: {
      throw std::runtime_error("The chosen manipulator model type is not supported!");
    }
  }  // end of switch-case
}

// explicit template instantiation
template class bolza::mobile_manipulator::MobileManipulatorPinocchioMappingTpl<bolza::scalar_t>;
template class bolza::mobile_manipulator::MobileManipulatorPinocchioMappingTpl<bolza::ad_scalar_t>;

}  // namespace mobile_manipulator
}  // namespace bolza
