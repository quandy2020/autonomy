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
 * @file self_collision_constraint_cpp_ad.hpp
 * @brief This class provides the CppAD variant of the Self-collision constraints. Therefore no pre-computation is required. The class has.
 */


#include <memory>

#include <bolza/core/constraint/state_constraint.hpp>
#include "bolza/pinocchio/interface/pinocchio_state_input_mapping.hpp"
#include "bolza/pinocchio/self_collision/self_collision_cpp_ad.hpp"

namespace bolza {

/**
 * This class provides the CppAD variant of the Self-collision constraints. Therefore no pre-computation is required. The class has
 * two constructors. The constructor with an additional argument, "updateCallback", is meant for cases that PinocchioStateInputMapping
 * requires extra update calls on PinocchioInterface, such as the centroidal model mapping (refer to CentroidalModelPinocchioMapping).
 *
 * See also SelfCollisionConstraint, which uses analytical computation and caching.
 */
class SelfCollisionConstraintCppAd final : public StateConstraint {
 public:
  using update_pinocchio_interface_callback =
      std::function<void(const vector_t& state, PinocchioInterfaceTpl<scalar_t>& pinocchioInterface)>;

  /**
   * Constructor
   *
   * @param [in] pinocchioInterface: Pinocchio interface of the robot model.
   * @param [in] mapping: The pinocchio mapping from pinocchio states to ocs2 states.
   * @param [in] pinocchioGeometryInterface: Pinocchio geometry interface of the robot model.
   * @param [in] minimumDistance: The minimum allowed distance between collision pairs.
   * @param [in] modelName: Name of the generated model library.
   * @param [in] modelFolder: Folder to save the model library files to.
   * @param [in] recompileLibraries: If true, the model library will be newly compiled. If false, an existing library will be loaded if
   *                                 available.
   * @param [in] verbose: If true, print information. Otherwise, no information is printed.
   */
  SelfCollisionConstraintCppAd(PinocchioInterface pinocchioInterface, const PinocchioStateInputMapping<scalar_t>& mapping,
                               PinocchioGeometryInterface pinocchioGeometryInterface, scalar_t minimumDistance,
                               const std::string& modelName, const std::string& modelFolder = "/tmp/ocs2", bool recompileLibraries = true,
                               bool verbose = true);

  /**
   * Constructor
   *
   * @param [in] pinocchioInterface: Pinocchio interface of the robot model.
   * @param [in] mapping: The pinocchio mapping from pinocchio states to ocs2 states.
   * @param [in] pinocchioGeometryInterface: Pinocchio geometry interface of the robot model.
   * @param [in] minimumDistance: The minimum allowed distance between collision pairs.
   * @param [in] updateCallback: In the cases that PinocchioStateInputMapping requires some additional update calls on PinocchioInterface,
   *                             use this callback (no need to call pinocchio::forwardKinematics).
   * @param [in] modelName: Name of the generated model library.
   * @param [in] modelFolder: Folder to save the model library files to.
   * @param [in] recompileLibraries: If true, the model library will be newly compiled. If false, an existing library will be loaded if
   *                                 available.
   * @param [in] verbose: If true, print information. Otherwise, no information is printed.
   */
  SelfCollisionConstraintCppAd(PinocchioInterface pinocchioInterface, const PinocchioStateInputMapping<scalar_t>& mapping,
                               PinocchioGeometryInterface pinocchioGeometryInterface, scalar_t minimumDistance,
                               update_pinocchio_interface_callback updateCallback, const std::string& modelName,
                               const std::string& modelFolder = "/tmp/ocs2", bool recompileLibraries = true, bool verbose = true);

  /**
   * @brief Destructor.
   */
  ~SelfCollisionConstraintCppAd() override = default;
  /**
   * @brief Clone.
   * @param [in] this: This.
   * @return Pointer; may be null.
   */
  SelfCollisionConstraintCppAd* Clone() const override { return new SelfCollisionConstraintCppAd(*this); }

  /**
   * @brief Get Num Constraints.
   * @param [in] time: Time.
   * @return Result value.
   */
  size_t GetNumConstraints(scalar_t time) const override;

  /**
   * @brief Get the self collision distance values.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @return Result value.
   */
  vector_t GetValue(scalar_t time, const vector_t& state, const PreComputation&) const override;

  /**
   * @brief Get the self collision distance approximation.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @return Result value.
   */
  VectorFunctionLinearApproximation GetLinearApproximation(scalar_t time, const vector_t& state, const PreComputation&) const override;

 private:
  /**
   * @brief Constructs a SelfCollisionConstraintCppAd.
   * @param [in] rhs: Rhs.
   */
  SelfCollisionConstraintCppAd(const SelfCollisionConstraintCppAd& rhs);

  mutable PinocchioInterface pinocchioInterface_;
  SelfCollisionCppAd selfCollision_;
  std::unique_ptr<PinocchioStateInputMapping<scalar_t>> mappingPtr_;
  update_pinocchio_interface_callback updateCallback_;
};

}  // namespace bolza
