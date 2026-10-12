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
 * @file pinocchio_end_effector_kinematics_cpp_ad.hpp
 * @brief This class provides the CppAD implementation the end-effector Kinematics based on pinocchio. No pre-computation.
 */


#include <functional>
#include <string>
#include <vector>

#include "bolza/pinocchio/interface/pinocchio_interface.hpp"
#include "bolza/pinocchio/interface/pinocchio_state_input_mapping.hpp"
#include <bolza/tools/end_effector/end_effector_kinematics.hpp>

#include <bolza/core/automatic_differentiation/cpp_ad_interface.hpp>

namespace bolza {

/**
 * This class provides the CppAD implementation the end-effector Kinematics based on pinocchio. No pre-computation
 * is required. The class has two constructors. The constructor with an additional argument, "updateCallback", is meant
 * for cases where PinocchioStateInputMapping requires some extra update calls on PinocchioInterface, such as the
 * centroidal model mapping (refer to CentroidalModelPinocchioMapping).
 *
 * See also PinocchioEndEffectorKinematics, which uses analytical computation and caching.
 */
class PinocchioEndEffectorKinematicsCppAd final : public EndEffectorKinematics<scalar_t> {
 public:
  using EndEffectorKinematics<scalar_t>::vector3_t;
  using EndEffectorKinematics<scalar_t>::matrix3x_t;
  using EndEffectorKinematics<scalar_t>::quaternion_t;
  using update_pinocchio_interface_callback =
      std::function<void(const ad_vector_t& state, PinocchioInterfaceTpl<ad_scalar_t>& pinocchioInterface)>;

  /** Constructor
   * @param [in] pinocchioInterface: pinocchio interface.
   * @param [in] mapping: mapping from OCS2 to pinocchio state.
   * @param [in] endEffectorIds: array of end effector names.
   * @param [in] stateDim: size of state vector
   * @param [in] inputDim: size of input vector
   * @param [in] modelName: name of the generate model library
   * @param [in] modelFolder: folder to save the model library files to
   * @param [in] recompileLibraries: If true, the model library will be newly compiled. If false, an existing library will be loaded if
   *                                  available.
   * @param [in] verbose: print information.
   */
  PinocchioEndEffectorKinematicsCppAd(const PinocchioInterface& pinocchioInterface, const PinocchioStateInputMapping<ad_scalar_t>& mapping,
                                      std::vector<std::string> endEffectorIds, size_t stateDim, size_t inputDim,
                                      const std::string& modelName, const std::string& modelFolder = "/tmp/ocs2",
                                      bool recompileLibraries = true, bool verbose = false);

  /** Constructor
   * @param [in] pinocchioInterface: pinocchio interface.
   * @param [in] mapping: mapping from OCS2 to pinocchio state.
   * @param [in] endEffectorIds: array of end effector names.
   * @param [in] stateDim: size of state vector
   * @param [in] inputDim: size of input vector
   * @param [in] updateCallback: In the cases that PinocchioStateInputMapping requires some additional update calls on PinocchioInterface,
   *                              use this callback.
   * @param [in] modelName: name of the generate model library
   * @param [in] modelFolder: folder to save the model library files to
   * @param [in] recompileLibraries: If true, the model library will be newly compiled. If false, an existing library will be loaded if
   *                                  available.
   * @param [in] verbose: print information.
   */
  PinocchioEndEffectorKinematicsCppAd(const PinocchioInterface& pinocchioInterface, const PinocchioStateInputMapping<ad_scalar_t>& mapping,
                                      std::vector<std::string> endEffectorIds, size_t stateDim, size_t inputDim,
                                      update_pinocchio_interface_callback updateCallback, const std::string& modelName,
                                      const std::string& modelFolder = "/tmp/ocs2", bool recompileLibraries = true, bool verbose = false);

  /**
   * @brief Destructor.
   */
  ~PinocchioEndEffectorKinematicsCppAd() override = default;
  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  PinocchioEndEffectorKinematicsCppAd* Clone() const override;
  PinocchioEndEffectorKinematicsCppAd& operator=(const PinocchioEndEffectorKinematicsCppAd&) = delete;

  /**
   * @brief Get Ids.
   * @return Result value.
   */
  const std::vector<std::string>& GetIds() const override;

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
  std::vector<vector3_t> GetVelocity(const vector_t& state, const vector_t& input) const override;
  /**
   * @brief Get Orientation Error.
   * @param [in] state: State.
   * @param [in] referenceOrientations: Reference Orientations.
   * @return Result value.
   */
  std::vector<vector3_t> GetOrientationError(const vector_t& state, const std::vector<quaternion_t>& referenceOrientations) const override;

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
                                                                                const vector_t& input) const override;
  /**
   * @brief Get Orientation Error Linear Approximation.
   * @param [in] state: State.
   * @param [in] referenceOrientations: Reference Orientations.
   * @return Result value.
   */
  std::vector<VectorFunctionLinearApproximation> GetOrientationErrorLinearApproximation(
      const vector_t& state, const std::vector<quaternion_t>& referenceOrientations) const override;

 private:
  /**
   * @brief Constructs a PinocchioEndEffectorKinematicsCppAd.
   * @param [in] rhs: Rhs.
   */
  PinocchioEndEffectorKinematicsCppAd(const PinocchioEndEffectorKinematicsCppAd& rhs);

  /**
   * @brief Get Position Cpp Ad.
   * @param [in,out] pinocchioInterfaceCppAd: Pinocchio Interface Cpp Ad.
   * @param [in] mapping: Mapping.
   * @param [in] state: State.
   * @return Result value.
   */
  ad_vector_t GetPositionCppAd(PinocchioInterfaceCppAd& pinocchioInterfaceCppAd, const PinocchioStateInputMapping<ad_scalar_t>& mapping,
                               const ad_vector_t& state);
  /**
   * @brief Get Velocity Cpp Ad.
   * @param [in,out] pinocchioInterfaceCppAd: Pinocchio Interface Cpp Ad.
   * @param [in] mapping: Mapping.
   * @param [in] state: State.
   * @param [in] input: Input.
   * @return Result value.
   */
  ad_vector_t GetVelocityCppAd(PinocchioInterfaceCppAd& pinocchioInterfaceCppAd, const PinocchioStateInputMapping<ad_scalar_t>& mapping,
                               const ad_vector_t& state, const ad_vector_t& input);
  /**
   * @brief Get Orientation Error Cpp Ad.
   * @param [in,out] pinocchioInterfaceCppAd: Pinocchio Interface Cpp Ad.
   * @param [in] mapping: Mapping.
   * @param [in] state: State.
   * @param [in] params: Params.
   * @return Result value.
   */
  ad_vector_t GetOrientationErrorCppAd(PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
                                       const PinocchioStateInputMapping<ad_scalar_t>& mapping, const ad_vector_t& state,
                                       const ad_vector_t& params);

  std::unique_ptr<CppAdInterface> positionCppAdInterfacePtr_;
  std::unique_ptr<CppAdInterface> velocityCppAdInterfacePtr_;
  std::unique_ptr<CppAdInterface> orientationErrorCppAdInterfacePtr_;

  const std::vector<std::string> endEffectorIds_;
  std::vector<size_t> endEffectorFrameIds_;
};

}  // namespace bolza
