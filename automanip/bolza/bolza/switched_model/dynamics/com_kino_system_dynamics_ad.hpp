#pragma once

/**
 * @file com_kino_system_dynamics_ad.hpp
 * @brief com kino system dynamics ad (bolza).
 */


#include <bolza/core/dynamics/system_dynamics_base_ad.hpp>

#include "bolza/switched_model/core/com_model_base.hpp"
#include "bolza/switched_model/core/kinematics_model_base.hpp"
#include "bolza/switched_model/core/model_settings.hpp"
#include "bolza/switched_model/core/switched_model.hpp"
#include "bolza/switched_model/dynamics/com_kino_dynamics_parameters.hpp"
#include "bolza/switched_model/logic/dynamics_parameters_synchronized_module.hpp"
#include "bolza/switched_model/logic/switched_model_mode_schedule_manager.hpp"

namespace switched_model {

class ComKinoSystemDynamicsAd : public bolza::SystemDynamicsBaseAD {
 public:
  using Base = bolza::SystemDynamicsBaseAD;

  using ad_com_model_t = ComModelBase<ad_scalar_t>;
  using ad_kinematic_model_t = KinematicsModelBase<ad_scalar_t>;
  using parameters_t = ComKinoSystemDynamicsParameters<scalar_t>;
  using ad_parameters_t = ComKinoSystemDynamicsParameters<ad_scalar_t>;

  /**
   * @brief Constructs a ComKinoSystemDynamicsAd.
   * @param [in] adKinematicModel: Ad Kinematic Model.
   * @param [in] adComModel: Ad Com Model.
   * @param [in] dynamicsParametersModule: Dynamics Parameters Module.
   * @param [in] settings: Settings.
   */
  explicit ComKinoSystemDynamicsAd(const ad_kinematic_model_t& adKinematicModel, const ad_com_model_t& adComModel,
                                   const DynamicsParametersSynchronizedModule& dynamicsParametersModule, ModelSettings settings);

  /**
   * @brief Constructs a ComKinoSystemDynamicsAd.
   * @param [in] rhs: Rhs.
   */
  ComKinoSystemDynamicsAd(const ComKinoSystemDynamicsAd& rhs);

  /**
   * @brief Destructor.
   */
  ~ComKinoSystemDynamicsAd() override = default;

  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  ComKinoSystemDynamicsAd* Clone() const override;

  /**
   * @brief System Flow Map.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] input: Input.
   * @param [in] parameters: Parameters.
   * @return Result value.
   */
  bolza::ad_vector_t SystemFlowMap(bolza::ad_scalar_t time, const bolza::ad_vector_t& state, const bolza::ad_vector_t& input,
                                  const bolza::ad_vector_t& parameters) const override;

  /**
   * @brief Get Flow Map Parameters.
   * @param [in] time: Time.
   * @return Result value.
   */
  bolza::vector_t GetFlowMapParameters(scalar_t time, const bolza::PreComputation& /* preComputation */) const override {
    /**
     * @brief Get Active Dynamics Parameters.
     * @return Result value.
     */
    return dynamicsParametersModulePtr_->getActiveDynamicsParameters().asVector();
  }

  /**
   * @brief Get Num Flow Map Parameters.
   * @return Result value.
   */
  size_t GetNumFlowMapParameters() const override { return parameters_t::GetNumParameters(); }

  template <typename SCALAR_T>
  /**
   * @brief Compute Com State Derivative.
   * @param [in] comModel: Com Model.
   * @param [in] kinematicsModel: Kinematics Model.
   * @param [in] comKinoState: Com Kino State.
   * @param [in] comKinoInput: Com Kino Input.
   * @param [in] parameters: Parameters.
   * @return Result value.
   */
  static com_state_s_t<SCALAR_T> ComputeComStateDerivative(
      const ComModelBase<SCALAR_T>& comModel, const KinematicsModelBase<SCALAR_T>& kinematicsModel,
      const comkino_state_s_t<SCALAR_T>& comKinoState, const comkino_input_s_t<SCALAR_T>& comKinoInput,
      const ComKinoSystemDynamicsParameters<SCALAR_T>& parameters = ComKinoSystemDynamicsParameters<SCALAR_T>());

 private:
  std::unique_ptr<ad_kinematic_model_t> adKinematicModelPtr_;
  std::unique_ptr<ad_com_model_t> adComModelPtr_;
  const DynamicsParametersSynchronizedModule* dynamicsParametersModulePtr_;
  ModelSettings settings_;
};

//! Explicit instantiations of computeComStateDerivative
/**
 * @brief Compute Com State Derivative.
 * @param [in] comModel: Com Model.
 * @param [in] kinematicsModel: Kinematics Model.
 * @param [in] comKinoState: Com Kino State.
 * @param [in] comKinoInput: Com Kino Input.
 * @param [in] parameters: Parameters.
 * @return Result value.
 */
extern template com_state_t ComKinoSystemDynamicsAd::ComputeComStateDerivative(const ComModelBase<scalar_t>& comModel,
                                                                               const KinematicsModelBase<scalar_t>& kinematicsModel,
                                                                               const comkino_state_t& comKinoState,
                                                                               const comkino_input_t& comKinoInput,
                                                                               const parameters_t& parameters);
/**
 * @brief Compute Com State Derivative.
 * @param [in] comModel: Com Model.
 * @param [in] kinematicsModel: Kinematics Model.
 * @param [in] comKinoState: Com Kino State.
 * @param [in] comKinoInput: Com Kino Input.
 * @param [in] parameters: Parameters.
 * @return Result value.
 */
extern template com_state_ad_t ComKinoSystemDynamicsAd::ComputeComStateDerivative(const ComModelBase<ad_scalar_t>& comModel,
                                                                                  const KinematicsModelBase<ad_scalar_t>& kinematicsModel,
                                                                                  const comkino_state_ad_t& comKinoState,
                                                                                  const comkino_input_ad_t& comKinoInput,
                                                                                  const ad_parameters_t& parameters);

}  // namespace switched_model
