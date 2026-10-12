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

#include <bolza/tools/common/rotation_derivatives_transforms.hpp>
#include <bolza/tools/common/rotation_transforms.hpp>
#include <bolza/tools/common/skew_symmetric_matrix.hpp>

namespace bolza {

/******************************************************************************************************/

/**
 * @file model_helper_functions_impl.hpp
 * @brief model helper functions impl (bolza).
 */

/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
/**
 * @brief Compute Floating Base Centroidal Momentum Matrix Inverse.
 * @param [in] Ab: Ab.
 * @return Result value.
 */
Eigen::Matrix<SCALAR_T, 6, 6> ComputeFloatingBaseCentroidalMomentumMatrixInverse(const Eigen::Matrix<SCALAR_T, 6, 6>& Ab) {
  const SCALAR_T mass = Ab(0, 0);
  Eigen::Matrix<SCALAR_T, 3, 3> Ab_22_inv = Ab.template block<3, 3>(3, 3).inverse();
  Eigen::Matrix<SCALAR_T, 6, 6> Ab_inv = Eigen::Matrix<SCALAR_T, 6, 6>::Zero();
  Ab_inv << 1.0 / mass * Eigen::Matrix<SCALAR_T, 3, 3>::Identity(), -1.0 / mass * Ab.template block<3, 3>(0, 3) * Ab_22_inv,
      Eigen::Matrix<SCALAR_T, 3, 3>::Zero(), Ab_22_inv;
  return Ab_inv;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
/**
 * @brief Get Mapping Zyx Gradient.
 * @param [in] eulerAngles: Euler Angles.
 * @return Result value.
 */
std::array<Eigen::Matrix<SCALAR_T, 3, 3>, 3> GetMappingZyxGradient(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles) {
  const SCALAR_T z = eulerAngles(0);
  const SCALAR_T y = eulerAngles(1);
  const SCALAR_T x = eulerAngles(2);

  // clang-format off
  Eigen::Matrix<SCALAR_T, 3, 3> dTdz, dTdy, dTdx;
  /**
   * @brief SCALAR T.
   * @return Result value.
   */
  dTdz << SCALAR_T(0),     -cos(z),        -cos(y)*sin(z),
          SCALAR_T(0),      -sin(z),        cos(y)*cos(z),
          SCALAR_T(0),     SCALAR_T(0),      SCALAR_T(0);

  /**
   * @brief SCALAR T.
   * @param [in] y: Y.
   * @return Result value.
   */
  dTdy << SCALAR_T(0),      SCALAR_T(0),        -sin(y)*cos(z),
          SCALAR_T(0),      SCALAR_T(0),        -sin(y)*sin(z),
          SCALAR_T(0),      SCALAR_T(0),          -cos(y);

  /**
   * @brief Set Zero.
   * @return Result value.
   */
  dTdx.setZero();
  // clang-format on

  return {dTdz, dTdy, dTdx};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
/**
 * @brief Get Rotation Matrix Zyx Gradient.
 * @param [in] eulerAngles: Euler Angles.
 * @return Result value.
 */
std::array<Eigen::Matrix<SCALAR_T, 3, 3>, 3> GetRotationMatrixZyxGradient(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles) {
  const SCALAR_T z = eulerAngles(0);
  const SCALAR_T y = eulerAngles(1);
  const SCALAR_T x = eulerAngles(2);

  const SCALAR_T c1 = cos(z);
  const SCALAR_T c2 = cos(y);
  const SCALAR_T c3 = cos(x);
  const SCALAR_T s1 = sin(z);
  const SCALAR_T s2 = sin(y);
  const SCALAR_T s3 = sin(x);

  const SCALAR_T dc1 = -s1;
  const SCALAR_T dc2 = -s2;
  const SCALAR_T dc3 = -s3;
  const SCALAR_T ds1 = c1;
  const SCALAR_T ds2 = c2;
  const SCALAR_T ds3 = c3;

  // clang-format off
  Eigen::Matrix<SCALAR_T, 3, 3> dRdz, dRdy, dRdx;
  dRdz << dc1 * c2,      dc1 * s2 * s3 - ds1 * c3,       dc1 * s2 * c3 + ds1 * s3,
          ds1 * c2,      ds1 * s2 * s3 + dc1 * c3,       ds1 * s2 * c3 - dc1 * s3,
          /**
           * @brief Constructs a SCALAR_T.
           */
          SCALAR_T(0),       SCALAR_T(0),                      SCALAR_T(0);

  dRdy << c1 * dc2,      c1 * ds2 * s3,       c1 * ds2 * c3,
          s1 * dc2,      s1 * ds2 * s3,       s1 * ds2 * c3,
           -ds2,            dc2 * s3,            dc2 * c3;

  dRdx << SCALAR_T(0),      c1 * s2 * ds3 - s1 * dc3,       c1 * s2 * dc3 + s1 * ds3,
          SCALAR_T(0),      s1 * s2 * ds3 + c1 * dc3,       s1 * s2 * dc3 - c1 * ds3,
          SCALAR_T(0),                c2 * ds3,                      c2 * dc3;
  // clang-format on

  return {dRdz, dRdy, dRdx};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
/**
 * @brief Get Centroidal Momentum Zyx Gradient.
 * @param [in] interface: Interface.
 * @param [in] info: Info.
 * @param [in] q: Q.
 * @param [in] v: V.
 * @return Result value.
 */
Eigen::Matrix<SCALAR_T, 6, 3> GetCentroidalMomentumZyxGradient(const PinocchioInterfaceTpl<SCALAR_T>& interface,
                                                               const CentroidalModelInfoTpl<SCALAR_T>& info,
                                                               const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& q,
                                                               const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& v) {
  using matrix_t = Eigen::Matrix<SCALAR_T, Eigen::Dynamic, Eigen::Dynamic>;
  using vector3_t = Eigen::Matrix<SCALAR_T, 3, 1>;
  using matrix3_t = Eigen::Matrix<SCALAR_T, 3, 3>;

  const auto& data = interface.GetData();
  const auto m = info.robotMass;
  const vector3_t eulerAngles = q.template segment<3>(3);
  const vector3_t eulerAnglesDerivatives = v.template segment<3>(3);
  const auto T = GetMappingFromEulerAnglesZyxDerivativeToGlobalAngularVelocity(eulerAngles);
  const auto R = GetRotationMatrixFromZyxEulerAngles(eulerAngles);
  matrix3_t Ibaseframe, Iworldframe;
  vector3_t rbaseframe, rworldframe;

  switch (info.centroidalModelType) {
    case CentroidalModelType::FullCentroidalDynamics: {
      Iworldframe = data.Ig.inertia().matrix();
      /**
       * @brief Noalias.
       * @return Result value.
       */
      Ibaseframe.noalias() = R.transpose() * (Iworldframe * R);
      rworldframe = q.template head<3>() - data.com[0];
      rbaseframe.noalias() = R.transpose() * rworldframe;
      break;
    }
    case CentroidalModelType::SingleRigidBodyDynamics: {
      Ibaseframe = info.centroidalInertiaNominal;
      /**
       * @brief Noalias.
       * @return Result value.
       */
      Iworldframe.noalias() = R * (Ibaseframe * R.transpose());
      rbaseframe = info.comToBasePositionNominal;
      rworldframe.noalias() = R * rbaseframe;
      break;
    }
    default: {
      /**
       * @brief Runtime error.
       * @return Result value.
       */
      throw std::runtime_error("The chosen centroidal model type is not supported.");
    }
  }

  const auto S = SkewSymmetricMatrix(rworldframe);
  const auto dT = GetMappingZyxGradient(eulerAngles);
  const auto dR = GetRotationMatrixZyxGradient(eulerAngles);

  std::array<matrix3_t, 3> dS;
  for (size_t i = 0; i < 3; i++) {
    const vector3_t dr = dR[i] * rbaseframe;
    dS[i] = SkewSymmetricMatrix(dr);
  }

  matrix_t dhdq = matrix_t::Zero(6, 3);
  for (size_t i = 0; i < 3; i++) {
    const vector3_t T_eulerAnglesDev = T * eulerAnglesDerivatives;
    const vector3_t dT_eulerAnglesDev = dT[i] * eulerAnglesDerivatives;
    const matrix3_t dR_I_Rtrans = dR[i] * Ibaseframe * R.transpose();

    /**
     * @brief Noalias.
     * @return Result value.
     */
    dhdq.template block<3, 1>(0, i).noalias() = m * (S * dT_eulerAnglesDev);
    /**
     * @brief Noalias.
     * @return Result value.
     */
    dhdq.template block<3, 1>(0, i).noalias() += m * (dS[i] * T_eulerAnglesDev);

    dhdq.template block<3, 1>(3, i).noalias() = (dR_I_Rtrans + dR_I_Rtrans.transpose()) * T_eulerAnglesDev;
    dhdq.template block<3, 1>(3, i).noalias() += Iworldframe * dT_eulerAnglesDev;
  }

  if (info.centroidalModelType == CentroidalModelType::FullCentroidalDynamics) {
    const auto jointVelocities = v.tail(info.actuatedDofNum);
    const vector3_t comLinearVelocityInWorldFrame = (1.0 / m) * (data.Ag.topRightCorner(3, info.actuatedDofNum) * jointVelocities);
    const vector3_t comAngularVelocityInWorldFrame =
        /**
         * @brief Inverse.
         * @param [in,out] jointVelocities: Joint Velocities.
         * @return Result value.
         */
        Iworldframe.inverse() * (data.Ag.bottomRightCorner(3, info.actuatedDofNum) * jointVelocities);
    const vector3_t linearMomentumInBaseFrame = m * (R.transpose() * comLinearVelocityInWorldFrame);
    const vector3_t angularMomentumInBaseFrame = Ibaseframe * (R.transpose() * comAngularVelocityInWorldFrame);
    for (size_t i = 0; i < 3; i++) {
      dhdq.template block<3, 1>(0, i).noalias() += dR[i] * linearMomentumInBaseFrame;
      dhdq.template block<3, 1>(3, i).noalias() += dR[i] * angularMomentumInBaseFrame;
    }
  }

  return dhdq;
}

}  // namespace bolza
