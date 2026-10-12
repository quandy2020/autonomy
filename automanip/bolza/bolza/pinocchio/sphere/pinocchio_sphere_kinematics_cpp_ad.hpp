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
 * @file pinocchio_sphere_kinematics_cpp_ad.hpp
 * @brief End-effector Kinematics implementation using pinocchio and CppAD.
 */


#include <string>
#include <vector>

#include "bolza/pinocchio/interface/pinocchio_interface.hpp"
#include "bolza/pinocchio/interface/pinocchio_state_input_mapping.hpp"
#include <bolza/tools/end_effector/end_effector_kinematics.hpp>
#include "bolza/pinocchio/sphere/pinocchio_sphere_interface.hpp"

#include <bolza/core/automatic_differentiation/cpp_ad_interface.hpp>

namespace bolza {

/**
 * End-effector Kinematics implementation using pinocchio and CppAD.
 *
 * @note See also PinocchioSphereKinematics, which uses analytical computation and caching.
 */
class PinocchioSphereKinematicsCppAd final : public EndEffectorKinematics<scalar_t> {
 public:
  using EndEffectorKinematics<scalar_t>::vector3_t;
  using EndEffectorKinematics<scalar_t>::matrix3x_t;
  using EndEffectorKinematics<scalar_t>::quaternion_t;

  struct SphereApproxParam {
    SphereApproxParam(vector3_t placementTranslation, quaternion_t placementOrientation, std::vector<vector3_t> sphereCentersToObjectCenter)
        /**
         * @brief Placement Translation.
         * @return Result value.
         */
        : PlacementTranslation(std::move(placementTranslation)),
          PlacementOrientation(std::move(placementOrientation)),
          SphereCentersToObjectCenter(std::move(sphereCentersToObjectCenter)) {}
    vector3_t placementTranslation;
    quaternion_t placementOrientation;
    std::vector<vector3_t> sphereCentersToObjectCenter;
  };

  /** Constructor
   * @param [in] pinocchioInterface: pinocchio interface.
   * @param [in] pinocchioSphereInterface: pinocchio sphere interface
   * @param [in] mapping: mapping from OCS2 to pinocchio state.
   * @param [in] stateDim: size of state vector
   * @param [in] inputDim: size of input vector
   * @param [in] modelName: name of the generate model library
   * @param [in] modelFolder: folder to save the model library files to
   * @param [in] recompileLibraries: If true, the model library will be newly compiled. If false, an existing library will be loaded if
   *                                  available.
   * @param [in] verbose: print information.
   */
  PinocchioSphereKinematicsCppAd(const PinocchioInterface& pinocchioInterface, PinocchioSphereInterface pinocchioSphereInterface,
                                 const PinocchioStateInputMapping<ad_scalar_t>& mapping, size_t stateDim, size_t inputDim,
                                 const std::string& modelName, const std::string& modelFolder = "/tmp/ocs2", bool recompileLibraries = true,
                                 bool verbose = false);

  /**
   * @brief Destructor.
   */
  ~PinocchioSphereKinematicsCppAd() override = default;
  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  PinocchioSphereKinematicsCppAd* Clone() const override;
  PinocchioSphereKinematicsCppAd& operator=(const PinocchioSphereKinematicsCppAd&) = delete;

  /**
   * @brief Get Pinocchio Sphere Interface.
   * @return Result value.
   */
  const PinocchioSphereInterface& GetPinocchioSphereInterface() const { return pinocchioSphereInterface_; };

  /**
   * @brief Get Ids.
   * @return Result value.
   */
  const std::vector<std::string>& GetIds() const override { return linkIds_; };

  /**
   * @brief Get Position.
   * @param [in] state: State.
   * @return Result value.
   */
  std::vector<vector3_t> GetPosition(const vector_t& state) const override;
  /**
   * @brief Get Velocity.
   * @param [in] state: State.
   * @param [in] input: Input.
   * @return Result value.
   */
  std::vector<vector3_t> GetVelocity(const vector_t& state, const vector_t& input) const override {
    /**
     * @brief Runtime error.
     * @return Result value.
     */
    throw std::runtime_error("[PinocchioSphereKinematicsCppAd] GetVelocity() is not implemented");
  };
  /**
   * @brief Get Orientation Error.
   * @param [in] state: State.
   * @param [in] referenceOrientations: Reference Orientations.
   * @return Result value.
   */
  std::vector<vector3_t> GetOrientationError(const vector_t& state, const std::vector<quaternion_t>& referenceOrientations) const override {
    /**
     * @brief Runtime error.
     * @return Result value.
     */
    throw std::runtime_error("[PinocchioSphereKinematicsCppAd] GetOrientationError() is not implemented");
  };

  /**
   * @brief Get Position Linear Approximation.
   * @param [in] state: State.
   * @return Result value.
   */
  std::vector<VectorFunctionLinearApproximation> GetPositionLinearApproximation(const vector_t& state) const override;
  /**
   * @brief Get Velocity Linear Approximation.
   * @param [in] state: State.
   * @param [in] input: Input.
   * @return Result value.
   */
  std::vector<VectorFunctionLinearApproximation> GetVelocityLinearApproximation(const vector_t& state,
                                                                                const vector_t& input) const override {
    /**
     * @brief Runtime error.
     * @return Result value.
     */
    throw std::runtime_error("[PinocchioSphereKinematicsCppAd] GetVelocityLinearApproximation() is not implemented");
  };
  /**
   * @brief Get Orientation Error Linear Approximation.
   * @param [in] state: State.
   * @param [in] referenceOrientations: Reference Orientations.
   * @return Result value.
   */
  std::vector<VectorFunctionLinearApproximation> GetOrientationErrorLinearApproximation(
      const vector_t& state, const std::vector<quaternion_t>& referenceOrientations) const override {
    /**
     * @brief Runtime error.
     * @return Result value.
     */
    throw std::runtime_error("[PinocchioSphereKinematicsCppAd] GetOrientationErrorLinearApproximation() is not implemented");
  };

 private:
  /**
   * @brief Constructs a PinocchioSphereKinematicsCppAd.
   * @param [in] rhs: Rhs.
   */
  PinocchioSphereKinematicsCppAd(const PinocchioSphereKinematicsCppAd& rhs);

  /**
   * @brief Create Sphere Approx Params.
   * @return Result value.
   */
  std::vector<SphereApproxParam> CreateSphereApproxParams() const;
  /**
   * @brief Get Position Cpp Ad.
   * @param [in,out] pinocchioInterfaceCppAd: Pinocchio Interface Cpp Ad.
   * @param [in] mapping: Mapping.
   * @param [in] sphereApproxParams: Sphere Approx Params.
   * @param [in] state: State.
   * @return Result value.
   */
  ad_vector_t GetPositionCppAd(PinocchioInterfaceCppAd& pinocchioInterfaceCppAd, const PinocchioStateInputMapping<ad_scalar_t>& mapping,
                               const std::vector<SphereApproxParam>& sphereApproxParams, const ad_vector_t& state);

  std::unique_ptr<CppAdInterface> positionCppAdInterfacePtr_;

  PinocchioSphereInterface pinocchioSphereInterface_;
  std::vector<std::string> linkIds_;
};

}  // namespace bolza
