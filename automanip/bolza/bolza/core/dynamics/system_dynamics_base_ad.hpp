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
 * @file system_dynamics_base_ad.hpp
 * @brief The system dynamics Base with Algorithmic Differentiation (i.e. Auto Differentiation).
 */


#include <bolza/core/automatic_differentiation/cpp_ad_interface.hpp>
#include <bolza/core/automatic_differentiation/types.hpp>
#include <bolza/core/dynamics/system_dynamics_base.hpp>

namespace bolza {

/**
 * The system dynamics Base with Algorithmic Differentiation (i.e. Auto Differentiation).
 * The linearized system flow map is defined as: \n
 * \f$ dx/dt = A(t) \delta x + B(t) \delta u \f$ \n
 * The linearized system jump map is defined as: \n
 * \f$ x^+ = G \delta x + H \delta u \f$ \n
 */
class SystemDynamicsBaseAD : public SystemDynamicsBase {
 public:
  /** Constructor */
  SystemDynamicsBaseAD();

  /** Default destructor */
  ~SystemDynamicsBaseAD() override = default;

  /**
   * Initializes model libraries
   *
   * @param [in] stateDim: state vector dimension.
   * @param [in] inputDim: input vector dimension.
   * @param [in] modelName: name of the generate model library
   * @param [in] modelFolder: folder to save the model library files to
   * @param [in] recompileLibraries: If true, always compile the model library, else try to load existing library if available.
   * @param [in] verbose: print information.
   */
  void Initialize(size_t stateDim, size_t inputDim, const std::string& modelName, const std::string& modelFolder = "/tmp/ocs2",
                  bool recompileLibraries = true, bool verbose = true);

  /**
   * @brief Compute Flow Map.
   * @param [in] t: T.
   * @param [in] x: X.
   * @param [in] u: U.
   * @param [in] preComputation: Pre Computation.
   * @return Result value.
   */
  vector_t ComputeFlowMap(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation& preComputation) final;

  /**
   * @brief Compute Jump Map.
   * @param [in] t: T.
   * @param [in] x: X.
   * @param [in] preComputation: Pre Computation.
   * @return Result value.
   */
  vector_t ComputeJumpMap(scalar_t t, const vector_t& x, const PreComputation& preComputation) final;

  /**
   * @brief Compute Guard Surfaces.
   * @param [in] t: T.
   * @param [in] x: X.
   * @return Result value.
   */
  vector_t ComputeGuardSurfaces(scalar_t t, const vector_t& x) final;

  /**
   * @brief Linear Approximation.
   * @param [in] t: T.
   * @param [in] x: X.
   * @param [in] u: U.
   * @param [in] preComputation: Pre Computation.
   * @return Result value.
   */
  VectorFunctionLinearApproximation LinearApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                        const PreComputation& preComputation) final;

  /**
   * @brief Jump Map Linear Approximation.
   * @param [in] t: T.
   * @param [in] x: X.
   * @param [in] preComputation: Pre Computation.
   * @return Result value.
   */
  VectorFunctionLinearApproximation JumpMapLinearApproximation(scalar_t t, const vector_t& x, const PreComputation& preComputation) final;

  /**
   * @brief Guard Surfaces Linear Approximation.
   * @param [in] t: T.
   * @param [in] x: X.
   * @param [in] u: U.
   * @return Result value.
   */
  VectorFunctionLinearApproximation GuardSurfacesLinearApproximation(scalar_t t, const vector_t& x, const vector_t& u) final;

    /**
   * @brief @note: Requires linear approximation to be called before 
   * @param [in] t: T.
   * @param [in] x: X.
   * @param [in] u: U.
   * @return Result value.
   */
  vector_t FlowMapDerivativeTime(scalar_t t, const vector_t& x, const vector_t& u) final;

    /**
   * @brief @note: Requires jump map linear approximation to be called before 
   * @param [in] t: T.
   * @param [in] x: X.
   * @param [in] u: U.
   * @return Result value.
   */
  vector_t JumpMapDerivativeTime(scalar_t t, const vector_t& x, const vector_t& u) final;

    /**
   * @brief @note: Requires guard surfaces linear approximation to be called before 
   * @param [in] t: T.
   * @param [in] x: X.
   * @param [in] u: U.
   * @return Result value.
   */
  vector_t GuardSurfacesDerivativeTime(scalar_t t, const vector_t& x, const vector_t& u) final;

 protected:
    /**
   * @brief Copy constructor 
   * @param [in] rhs: Rhs.
   */
  SystemDynamicsBaseAD(const SystemDynamicsBaseAD& rhs);

  /**
   * Interface method to the state flow map of the hybrid system. This method should be implemented by the derived class.
   *
   * @param [in] time: time.
   * @param [in] state: state vector.
   * @param [in] input: input vector.
   * @param [in] parameters: parameter vector.
   * @return state vector time derivative.
   */
  virtual ad_vector_t SystemFlowMap(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                                    const ad_vector_t& parameters) const = 0;

  /**
   * Gets the parameters of the system flow map
   *
   * @param [in] time: Current time.
   * @return The parameters to be set in the flow map at the start of the horizon
   */
  virtual vector_t GetFlowMapParameters(scalar_t time, const PreComputation& /* preComputation */) const { /**
  * @brief Vector t.
  * @return Result value.
  */
 return vector_t(0); }

  /**
   * Number of parameters for system flow map.
   *
   * @return number of parameters
   */
  virtual size_t GetNumFlowMapParameters() const { return 0; }

  /**
   * Interface method to the state jump map of the hybrid system. This method can be implemented by the derived class.
   *
   * @param [in] time: time.
   * @param [in] state: state vector.
   * @param [in] parameters: parameter vector.
   * @return jumped state.
   */
  virtual ad_vector_t SystemJumpMap(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& parameters) const;

  /**
   * Gets the parameters of the jump map
   *
   * @param [in] time: Current time.
   * @return The parameters to be set in the jump map
   */
  virtual vector_t GetJumpMapParameters(scalar_t time, const PreComputation& /* preComputation */) const { /**
  * @brief Vector t.
  * @return Result value.
  */
 return vector_t(0); }

  /**
   * Number of parameters for jump map.
   *
   * @return number of parameters
   */
  virtual size_t GetNumJumpMapParameters() const { return 0; }

  /**
   * Interface method to the guard surfaces. This method can be implemented by the derived class.
   *
   * @param [in] time: time.
   * @param [in] state: state.
   * @param [in] input: input vector
   * @param [in] parameters: parameter vector.
   * @return A vector of guard surfaces values
   */
  virtual ad_vector_t SystemGuardSurfaces(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& parameters) const;

  /**
   * Gets the parameters of the guard surfaces
   *
   * @param [in] time: Current time.
   * @return The parameters to be set in the guard surfaces
   */
  virtual vector_t GetGuardSurfacesParameters(scalar_t time) const { /**
  * @brief Vector t.
  * @return Result value.
  */
 return vector_t(0); }

  /**
   * Number of parameters for guard surfaces.
   *
   * @return number of parameters
   */
  virtual size_t GetNumGuardSurfacesParameters() const { return 0; }

 private:
  std::unique_ptr<CppAdInterface> flowMapADInterfacePtr_;
  std::unique_ptr<CppAdInterface> jumpMapADInterfacePtr_;
  std::unique_ptr<CppAdInterface> guardSurfacesADInterfacePtr_;

  vector_t tapedTimeStateInput_;
  vector_t tapedTimeState_;

  /** Cached jacobians for time derivative */
  matrix_t flowJacobian_;
  matrix_t jumpJacobian_;
  matrix_t guardJacobian_;
};

}  // namespace bolza
