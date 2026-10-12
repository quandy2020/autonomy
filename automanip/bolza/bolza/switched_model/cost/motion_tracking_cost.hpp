//
// Created by rgrandia on 30.04.21.
//

#pragma once

/**
 * @file motion_tracking_cost.hpp
 * @brief Defines a cost over tracking base and feet references.
 */


#include <bolza/core/cost/state_input_gauss_newton_cost_ad.hpp>

#include "bolza/switched_model/core/com_model_base.hpp"
#include "bolza/switched_model/core/kinematics_model_base.hpp"
#include "bolza/switched_model/core/model_settings.hpp"
#include "bolza/switched_model/core/switched_model.hpp"
#include "bolza/switched_model/logic/switched_model_mode_schedule_manager.hpp"

namespace switched_model {

/**
 * Defines a cost over tracking base and feet references
 */
class MotionTrackingCost final : public bolza::StateInputCostGaussNewtonAd {
 public:
  /**
   * Cost settings per motion target, weights are applied as: cost = sum_i {w_i * (target_i - ref_i)^2}
   * Magic values set as default
   */
  struct Weights {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    vector3_t eulerXYZ{100.0, 200.0, 200.0};
    vector3_t comPosition{1000.0, 1000.0, 1500.0};
    vector3_t comAngularVelocity{5.0, 10.0, 10.0};
    vector3_t comLinearVelocity{15.0, 15.0, 30.0};
    feet_array_t<vector3_t> jointPosition = ConstantFeetArray(vector3_t(2.0, 2.0, 1.0));
    feet_array_t<vector3_t> footPosition = ConstantFeetArray(vector3_t(60.0, 60.0, 60.0));
    feet_array_t<vector3_t> jointVelocity = ConstantFeetArray(vector3_t(0.02, 0.02, 0.01));
    feet_array_t<vector3_t> footVelocity = ConstantFeetArray(vector3_t(1.0, 1.0, 1.0));
    feet_array_t<vector3_t> contactForce = ConstantFeetArray(vector3_t(0.001, 0.001, 0.001));
  };

  using kinematic_model_t = KinematicsModelBase<bolza::scalar_t>;
  using ad_kinematic_model_t = KinematicsModelBase<bolza::ad_scalar_t>;
  using com_model_t = ComModelBase<bolza::scalar_t>;
  using ad_com_model_t = ComModelBase<bolza::ad_scalar_t>;

  /**
   * @brief Constructs a MotionTrackingCost.
   * @param [in] settings: Settings.
   * @param [in] adKinematicModel: Ad Kinematic Model.
   * @param [in] modelSettings: Model Settings.
   */
  MotionTrackingCost(const Weights& settings, const ad_kinematic_model_t& adKinematicModel, const ModelSettings& modelSettings);

  /**
   * @brief Destructor.
   */
  ~MotionTrackingCost() override = default;
  /**
   * @brief Clone.
   * @param [in] this: This.
   * @return Pointer; may be null.
   */
  MotionTrackingCost* Clone() const { return new MotionTrackingCost(*this); }

  /**
   * @brief Get Parameters.
   * @param [in] time: Time.
   * @param [in] targetTrajectories: Target Trajectories.
   * @param [in] preComputation: Pre Computation.
   * @return Result value.
   */
  bolza::vector_t GetParameters(bolza::scalar_t time, const bolza::TargetTrajectories& targetTrajectories,
                               const bolza::PreComputation& preComputation) const override;

 protected:
  /**
   * @brief Constructs a MotionTrackingCost.
   * @param [in] other: Other.
   */
  MotionTrackingCost(const MotionTrackingCost& other);

  /**
   * @brief Cost Vector Function.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] input: Input.
   * @param [in] parameters: Parameters.
   * @return Result value.
   */
  bolza::ad_vector_t CostVectorFunction(bolza::ad_scalar_t time, const bolza::ad_vector_t& state, const bolza::ad_vector_t& input,
                                       const bolza::ad_vector_t& parameters) const override;

 private:
  std::unique_ptr<ad_kinematic_model_t> adKinematicModelPtr_;
  bolza::vector_t sqrtWeights_;
  const ModelSettings& modelSettings_;
};

/**
 * @brief Load Weights From File.
 * @param [in] filename: Filename.
 * @param [in] fieldname: Fieldname.
 * @param [in] verbose: Verbose.
 * @return Result value.
 */
MotionTrackingCost::Weights LoadWeightsFromFile(const std::string& filename, const std::string& fieldname, bool verbose = true);

}  // namespace switched_model
