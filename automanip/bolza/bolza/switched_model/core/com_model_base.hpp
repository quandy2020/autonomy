#pragma once

/**
 * @file com_model_base.hpp
 * @brief CoM Model Base Class.
 */


#include <bolza/switched_model/core/switched_model.hpp>

namespace switched_model {

/**
 * CoM Model Base Class
 */
template <typename SCALAR_T>
class ComModelBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Constructs a ComModelBase.
   */
  ComModelBase() = default;

  /**
   * @brief Destructor.
   */
  virtual ~ComModelBase() = default;

  /**
   * @brief Clone ComModelBase class.
   * @return Pointer; may be null.
   */
  virtual ComModelBase<SCALAR_T>* Clone() const = 0;

  /**
   * Total mass of robot
   * @return mass in kg
   */
  virtual SCALAR_T TotalMass() const = 0;

  /**
   * Computes the base acceleration based on the top 6 rows of the full rigid body dynamics of the system.
   *
   * @param [in] basePose: {EulerXYZ (3x1), base position in Origin frame (3x1)}
   * @param [in] baseLocalVelocities: {base angular velocity in Base Frame (3x1), base linear velocity in Base Frame (3x1)}
   * @param [in] jointPositions: Joint positions (12x1)
   * @param [in] jointVelocities: Joint velocities (12x1)
   * @param [in] jointAccelerations: Joint accelerations (12x1)
   * @param [in] forcesOnBaseInBaseFrame: { torque on base in Base Frame (3x1), linear forces on the base in Base Frame (3x1) }.
   * Forces on the base due to contact forces, i.e. J^T * F should also be included here.
   * @return baseLocalAccelerations : {base angular acceleration in Base Frame (3x1), base linear acceleration in Base Frame (3x1)}
   */
  virtual base_coordinate_s_t<SCALAR_T> CalculateBaseLocalAccelerations(
      const base_coordinate_s_t<SCALAR_T>& basePose, const base_coordinate_s_t<SCALAR_T>& baseLocalVelocities,
      const joint_coordinate_s_t<SCALAR_T>& jointPositions, const joint_coordinate_s_t<SCALAR_T>& jointVelocities,
      const joint_coordinate_s_t<SCALAR_T>& jointAccelerations,
      const switched_model::base_coordinate_s_t<SCALAR_T>& forcesOnBaseInBaseFrame) const = 0;
};

extern template class ComModelBase<scalar_t>;
extern template class ComModelBase<bolza::CppAdInterface::ad_scalar_t>;

/**
 * @brief Distribute the weight equally among all stance feet.
 * @param [in] comModel: Com Model.
 * @param [in] contactFlags: Contact Flags.
 * @param [in] baseOrientation: Base Orientation.
 * @return model inputs with {Forces in base, joint velocies}
 */
comkino_input_t WeightCompensatingInputs(const ComModelBase<scalar_t>& comModel, const contact_flag_t& contactFlags,
                                         const vector3_t& baseOrientation);

/**
 * @brief Weight Compensating Inputs.
 * @param [in] mass: Mass.
 * @param [in] contactFlags: Contact Flags.
 * @param [in] baseOrientation: Base Orientation.
 * @return Result value.
 */
comkino_input_t WeightCompensatingInputs(scalar_t mass, const contact_flag_t& contactFlags, const vector3_t& baseOrientation);

}  // end of namespace switched_model
