#pragma once

/**
 * @file dynamics_parameters_synchronized_module.hpp
 * @brief dynamics parameters synchronized module (bolza).
 */


#include <bolza/core/thread_support/synchronized.hpp>
#include <bolza/oc/synchronized_module/solver_synchronized_module.hpp>

#include "bolza/switched_model/dynamics/com_kino_dynamics_parameters.hpp"

namespace switched_model {

class DynamicsParametersSynchronizedModule : public bolza::SolverSynchronizedModule {
 public:
  /**
   * @brief Constructs a DynamicsParametersSynchronizedModule.
   */
  DynamicsParametersSynchronizedModule();

  /**
   * @brief Pre Solver Run.
   * @param [in] initTime: Init Time.
   * @param [in] finalTime: Final Time.
   * @param [in] initState: Init State.
   * @param [in] referenceManager: Reference Manager.
   */
  void PreSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                    const bolza::ReferenceManagerInterface& referenceManager) override;

  /**
   * @brief Post Solver Run.
   * @param [in] primalSolution: Primal Solution.
   */
  void PostSolverRun(const bolza::PrimalSolution& primalSolution) override{};

  // Write-able access to dynamics parameters
  /**
   * @brief Get Dynamics Parameters.
   * @return Result value.
   */
  bolza::Synchronized<ComKinoSystemDynamicsParameters<scalar_t>>& GetDynamicsParameters() { return newDynamicsParameters_; }
  /**
   * @brief Get Dynamics Parameters.
   * @return Result value.
   */
  const bolza::Synchronized<ComKinoSystemDynamicsParameters<scalar_t>>& GetDynamicsParameters() const { return newDynamicsParameters_; }

  // Read-only access to active dynamics parameters (Not thread safe while MPC is running!)
  /**
   * @brief Get Active Dynamics Parameters.
   * @return Result value.
   */
  const ComKinoSystemDynamicsParameters<scalar_t>& GetActiveDynamicsParameters() const { return activeDynamicsParameters_; }

 private:
  //! Parameters active in the current MPC optimization
  ComKinoSystemDynamicsParameters<scalar_t> activeDynamicsParameters_;
  //! Updated externally, becomes active in next MPC iteration
  bolza::Synchronized<ComKinoSystemDynamicsParameters<scalar_t>> newDynamicsParameters_;
};

}  // namespace switched_model
