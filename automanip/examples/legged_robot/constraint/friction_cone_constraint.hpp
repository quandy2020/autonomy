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

#include <bolza/pinocchio/centroidal/centroidal_model_info.hpp>
#include <bolza/core/constraint/state_input_constraint.hpp>

#include "examples/legged_robot/common/types.hpp"
#include "examples/legged_robot/reference_manager/switched_model_reference_manager.hpp"

namespace bolza {
namespace legged_robot {

/**
 * Implements the constraint h(t,x,u) >= 0
 *
 * frictionCoefficient * (Fz + gripperForce) - sqrt(Fx * Fx + Fy * Fy + regularization) >= 0
 *
 * The gripper force shifts the origin of the friction cone down in z-direction by the amount of gripping force available. This makes it
 * possible to produce tangential forces without applying a regular normal force on that foot, or to "pull" on the foot with magnitude up to
 * the gripping force.
 *
 * The regularization prevents the constraint gradient / hessian to go to infinity when Fx = Fz = 0. It also creates a parabolic safety
 * margin to the friction cone. For example: when Fx = Fy = 0 the constraint zero-crossing will be at Fz = 1/frictionCoefficient *
 * sqrt(regularization) instead of Fz = 0
 *
 */
class FrictionConeConstraint final : public StateInputConstraint {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * frictionCoefficient: The coefficient of friction.
   * regularization: A positive number to regulize the friction constraint. refer to the FrictionConeConstraint documentation.
   * gripperForce: Gripper force in normal direction.
   * hessianDiagonalShift: The Hessian shift to assure a strictly-convex quadratic constraint approximation.
   */
  struct Config {
    explicit Config(scalar_t frictionCoefficientParam = 0.7, scalar_t regularizationParam = 25.0, scalar_t gripperForceParam = 0.0,
                    scalar_t hessianDiagonalShiftParam = 1e-6)
        : frictionCoefficient(frictionCoefficientParam),
          regularization(regularizationParam),
          gripperForce(gripperForceParam),
          hessianDiagonalShift(hessianDiagonalShiftParam) {
      assert(frictionCoefficient > 0.0);
      assert(regularization > 0.0);
      assert(hessianDiagonalShift >= 0.0);
    }

    scalar_t frictionCoefficient;
    scalar_t regularization;
    scalar_t gripperForce;
    scalar_t hessianDiagonalShift;
  };

  /**
   * Constructor
   * @param [in] referenceManager : Switched model ReferenceManager.
   * @param [in] config : Friction model settings.
   * @param [in] contactPointIndex : The 3 DoF contact index.
   * @param [in] info : The centroidal model information.
   */
  FrictionConeConstraint(const SwitchedModelReferenceManager& referenceManager, Config config, size_t contactPointIndex,
                         CentroidalModelInfo info);

  ~FrictionConeConstraint() override = default;
  FrictionConeConstraint* Clone() const override { return new FrictionConeConstraint(*this); }

  bool IsActive(scalar_t time) const override;
  size_t GetNumConstraints(scalar_t time) const override { return 1; };
  vector_t GetValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const override;
  VectorFunctionLinearApproximation GetLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                           const PreComputation& preComp) const override;
  VectorFunctionQuadraticApproximation GetQuadraticApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                 const PreComputation& preComp) const override;

  /** Sets the estimated terrain normal expressed in the world frame. */
  void SetSurfaceNormalInWorld(const vector3_t& surfaceNormalInWorld);

 private:
  struct LocalForceDerivatives {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    matrix3_t dF_du;  // derivative local force w.r.t. forces in world frame
  };

  struct ConeLocalDerivatives {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    vector3_t dCone_dF;    // derivative w.r.t local force
    matrix3_t d2Cone_dF2;  // second derivative w.r.t local force
  };

  struct ConeDerivatives {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    vector3_t dCone_du;
    matrix3_t d2Cone_du2;
  };

  FrictionConeConstraint(const FrictionConeConstraint& other) = default;
  vector_t ConeConstraint(const vector3_t& localForces) const;
  LocalForceDerivatives ComputeLocalForceDerivatives(const vector3_t& forcesInBodyFrame) const;
  ConeLocalDerivatives ComputeConeLocalDerivatives(const vector3_t& localForces) const;
  ConeDerivatives ComputeConeConstraintDerivatives(const ConeLocalDerivatives& coneLocalDerivatives,
                                                   const LocalForceDerivatives& localForceDerivatives) const;

  matrix_t FrictionConeInputDerivative(size_t inputDim, const ConeDerivatives& coneDerivatives) const;
  matrix_t FrictionConeSecondDerivativeInput(size_t inputDim, const ConeDerivatives& coneDerivatives) const;
  matrix_t FrictionConeSecondDerivativeState(size_t stateDim, const ConeDerivatives& coneDerivatives) const;

  const SwitchedModelReferenceManager* referenceManagerPtr_;

  const Config config_;
  const size_t contactPointIndex_;
  const CentroidalModelInfo info_;

  matrix3_t worldToTerrain() const;

  // rotation world to terrain, used when setSurfaceNormalInWorld() is called
  matrix3_t t_R_w = matrix3_t::Identity();
  bool override_normal_ = false;
};

}  // namespace legged_robot
}  // namespace bolza
