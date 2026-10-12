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

#pragma once

/**
 * @file centroidal_model_info.hpp
 * @brief Casts CentroidalModelInfo to CentroidalModelInfoCppAD.
 */


#include <ostream>
#include <string>
#include <type_traits>

#include <bolza/core/types.hpp>
#include <bolza/core/automatic_differentiation/types.hpp>

namespace bolza {

template <typename SCALAR>
class CentroidalModelInfoTpl;

using CentroidalModelInfo = CentroidalModelInfoTpl<scalar_t>;
using CentroidalModelInfoCppAd = CentroidalModelInfoTpl<ad_scalar_t>;

enum class CentroidalModelType { FullCentroidalDynamics, SingleRigidBodyDynamics };

/**
 * @brief To String.
 * @param [in] type: Type.
 * @return Result value.
 */
std::string ToString(CentroidalModelType type);
std::ostream& operator<<(std::ostream& os, CentroidalModelType type);

template <typename SCALAR>
struct CentroidalModelInfoTpl {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using scalar_t = SCALAR;
  using vector_t = Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>;
  using vector3_t = Eigen::Matrix<SCALAR, 3, 1>;
  using matrix3_t = Eigen::Matrix<SCALAR, 3, 3>;

  template <typename T>  // Template for conditional compilation using SFINAE
  using EnableIfScalar_t = typename std::enable_if<std::is_same<T, scalar_t>::Value, bool>::type;

  CentroidalModelType centroidalModelType;      // full centroidal dynamics OR single rigid body dynamics (SRBD)
  size_t numThreeDofContacts;                   // 3DOF contacts, force only
  size_t numSixDofContacts;                     // 6DOF contacts, force and torque
  std::vector<size_t> endEffectorFrameIndices;  // indices of end-effector frames [3DOF contacts, 6DOF contacts]
  size_t generalizedCoordinatesNum;             // number of generalized coordinates in the pinocchio model
  size_t actuatedDofNum;                        // number of actuated degrees of freedom
  size_t stateDim;                              // number of states needed to define the system flow map
  size_t inputDim;                              // number of inputs needed to define the system flow map
  scalar_t robotMass;                           // total robot mass
  vector_t qPinocchioNominal;                   // nominal robot configuration used in the SRBD model
  matrix3_t centroidalInertiaNominal;           // nominal robot centroidal inertia used in the SRBD model (expressed in nominal base frame)
  vector3_t comToBasePositionNominal;           // nominal CoM to base position used in the SRBD model (expressed in nominal base frame)

  /** Casts CentroidalModelInfo to CentroidalModelInfoCppAD. */
  template <typename T = SCALAR, EnableIfScalar_t<T> = true>
  /**
   * @brief To Cpp Ad.
   * @return Result value.
   */
  CentroidalModelInfoCppAd ToCppAd() const;
};

/* Explicit template instantiation for scalar_t and ad_scalar_t */
extern template struct CentroidalModelInfoTpl<scalar_t>;
extern template struct CentroidalModelInfoTpl<ad_scalar_t>;
}  // namespace bolza
