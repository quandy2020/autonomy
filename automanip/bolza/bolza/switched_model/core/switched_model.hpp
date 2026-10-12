#pragma once

/**
 * @file switched_model.hpp
 * @brief Switched model definition:.
 */


#include <Eigen/Dense>
#include <array>

#include <bolza/core/types.hpp>
#include <bolza/core/automatic_differentiation/cpp_ad_interface.hpp>
#include <bolza/core/automatic_differentiation/types.hpp>

namespace switched_model {
/**
 * Switched model definition:
 *
 * state = [theta, p, w, v, q (4x)]
 * theta: EulerXYZ (3x1)
 * p: Base position in Origin frame (3x1)
 * w: Base angular velocity in Base Frame (3x1)
 * v: Base linear velocity in Base Frame (3x1)
 * q: Joint angles per leg [HAA, HFE, KFE] (3x1) [4x]
 *
 * input = [lambda (4x), qj (4x)]
 * lambda: Force at the EE [LF, RF, LH, RH] in Base Frame (3x1)
 * qj: Joint velocities per leg [HAA, HFE, KFE] (3x1)
 */

constexpr size_t NUM_CONTACT_POINTS = 4;
constexpr size_t BASE_COORDINATE_SIZE = 6;
constexpr size_t JOINT_COORDINATE_SIZE = 12;
constexpr size_t STATE_DIM = 2 * BASE_COORDINATE_SIZE + JOINT_COORDINATE_SIZE;  // 24
constexpr size_t INPUT_DIM = 3 * NUM_CONTACT_POINTS + JOINT_COORDINATE_SIZE;    // 24

/* Import ocs2 types into the switched_model namespace */
using bolza::ad_matrix_t;
using bolza::ad_scalar_t;
using bolza::ad_vector_t;
using bolza::matrix_array_t;
using bolza::matrix_t;
using bolza::scalar_array_t;
using bolza::scalar_t;
using bolza::size_array_t;
using bolza::vector_array_t;
using bolza::vector_t;

using bolza::ScalarFunctionQuadraticApproximation;
using bolza::VectorFunctionLinearApproximation;
using bolza::VectorFunctionQuadraticApproximation;

/* Define fixed-size types */
using state_vector_t = Eigen::Matrix<scalar_t, STATE_DIM, 1>;
using input_vector_t = Eigen::Matrix<scalar_t, INPUT_DIM, 1>;

/* Feet related declarations */
enum class FeetEnum { LF, RF, LH, RH };
template <typename T>
using feet_array_t = std::array<T, NUM_CONTACT_POINTS>;  // Fixed size container per foot
const feet_array_t<std::string> feetNames{"LF", "RF", "LH", "RH"};
using contact_flag_t = feet_array_t<bool>;  // Contact state per foot, true = in contact, false = not in contact

template <typename SCALAR_T>
using vector2_s_t = Eigen::Matrix<SCALAR_T, 2, 1>;
using vector2_t = vector2_s_t<scalar_t>;
using vector2_ad_t = vector2_s_t<bolza::CppAdInterface::ad_scalar_t>;

template <typename SCALAR_T>
using vector3_s_t = Eigen::Matrix<SCALAR_T, 3, 1>;
using vector3_t = vector3_s_t<scalar_t>;
using vector3_ad_t = vector3_s_t<bolza::CppAdInterface::ad_scalar_t>;

template <typename SCALAR_T>
using matrix3_s_t = Eigen::Matrix<SCALAR_T, 3, 3>;
using matrix3_t = matrix3_s_t<scalar_t>;
using matrix3_ad_t = matrix3_s_t<bolza::CppAdInterface::ad_scalar_t>;

template <typename SCALAR_T>
using vector6_s_t = Eigen::Matrix<SCALAR_T, 6, 1>;
using vector6_t = vector6_s_t<scalar_t>;
using vector6_ad_t = vector6_s_t<bolza::CppAdInterface::ad_scalar_t>;

template <typename SCALAR_T>
using matrix6_s_t = Eigen::Matrix<SCALAR_T, 6, 6>;
using matrix6_t = matrix6_s_t<scalar_t>;
using matrix6_ad_t = matrix6_s_t<bolza::CppAdInterface::ad_scalar_t>;

template <typename SCALAR_T>
using base_coordinate_s_t = Eigen::Matrix<SCALAR_T, BASE_COORDINATE_SIZE, 1>;
using base_coordinate_t = base_coordinate_s_t<scalar_t>;
using base_coordinate_ad_t = base_coordinate_s_t<bolza::CppAdInterface::ad_scalar_t>;

template <typename SCALAR_T>
using joint_coordinate_s_t = Eigen::Matrix<SCALAR_T, JOINT_COORDINATE_SIZE, 1>;
using joint_coordinate_t = joint_coordinate_s_t<scalar_t>;
using joint_coordinate_ad_t = joint_coordinate_s_t<bolza::CppAdInterface::ad_scalar_t>;

template <typename SCALAR_T>
using com_state_s_t = Eigen::Matrix<SCALAR_T, 2 * BASE_COORDINATE_SIZE, 1>;
using com_state_t = com_state_s_t<scalar_t>;
using com_state_ad_t = com_state_s_t<bolza::CppAdInterface::ad_scalar_t>;

template <typename SCALAR_T>
using comkino_state_s_t = Eigen::Matrix<SCALAR_T, STATE_DIM, 1>;
using comkino_state_t = comkino_state_s_t<scalar_t>;
using comkino_state_ad_t = comkino_state_s_t<bolza::CppAdInterface::ad_scalar_t>;

template <typename SCALAR_T>
using comkino_input_s_t = Eigen::Matrix<SCALAR_T, INPUT_DIM, 1>;
using comkino_input_t = comkino_input_s_t<scalar_t>;
using comkino_input_ad_t = comkino_input_s_t<bolza::CppAdInterface::ad_scalar_t>;

template <typename SCALAR_T>
/**
 * @brief Get Base Pose.
 * @param [in] comkinoState: Comkino State.
 * @return Result value.
 */
base_coordinate_s_t<SCALAR_T> GetBasePose(const Eigen::Matrix<SCALAR_T, -1, 1>& comkinoState) {
  return comkinoState.template head<BASE_COORDINATE_SIZE>();
}

template <typename SCALAR_T>
/**
 * @brief Get Base Pose.
 * @param [in] comkinoState: Comkino State.
 * @return Result value.
 */
base_coordinate_s_t<SCALAR_T> GetBasePose(const comkino_state_s_t<SCALAR_T>& comkinoState) {
  return comkinoState.template head<BASE_COORDINATE_SIZE>();
}

template <typename SCALAR_T>
/**
 * @brief Get Orientation.
 * @param [in] baseCoordinate: Base Coordinate.
 * @return Result value.
 */
vector3_s_t<SCALAR_T> GetOrientation(const base_coordinate_s_t<SCALAR_T>& baseCoordinate) {
  return baseCoordinate.template head<3>();
}

template <typename SCALAR_T>
/**
 * @brief Get Position In Origin.
 * @param [in] baseCoordinate: Base Coordinate.
 * @return Result value.
 */
vector3_s_t<SCALAR_T> GetPositionInOrigin(const base_coordinate_s_t<SCALAR_T>& baseCoordinate) {
  return baseCoordinate.template tail<3>();
}

template <typename SCALAR_T>
/**
 * @brief Get Base Local Velocities.
 * @param [in] comkinoState: Comkino State.
 * @return Result value.
 */
base_coordinate_s_t<SCALAR_T> GetBaseLocalVelocities(const Eigen::Matrix<SCALAR_T, -1, 1>& comkinoState) {
  return comkinoState.template segment<BASE_COORDINATE_SIZE>(BASE_COORDINATE_SIZE);
}

template <typename SCALAR_T>
/**
 * @brief Get Base Local Velocities.
 * @param [in] comkinoState: Comkino State.
 * @return Result value.
 */
base_coordinate_s_t<SCALAR_T> GetBaseLocalVelocities(const comkino_state_s_t<SCALAR_T>& comkinoState) {
  return comkinoState.template segment<BASE_COORDINATE_SIZE>(BASE_COORDINATE_SIZE);
}

template <typename SCALAR_T>
/**
 * @brief Get Angular Velocity.
 * @param [in] baseTwist: Base Twist.
 * @return Result value.
 */
vector3_s_t<SCALAR_T> GetAngularVelocity(const base_coordinate_s_t<SCALAR_T>& baseTwist) {
  return baseTwist.template head<3>();
}

template <typename SCALAR_T>
/**
 * @brief Get Angular Acceleration.
 * @param [in] baseAcceleration: Base Acceleration.
 * @return Result value.
 */
vector3_s_t<SCALAR_T> GetAngularAcceleration(const base_coordinate_s_t<SCALAR_T>& baseAcceleration) {
  return baseAcceleration.template head<3>();
}

template <typename SCALAR_T>
/**
 * @brief Get Linear Velocity.
 * @param [in] baseTwist: Base Twist.
 * @return Result value.
 */
vector3_s_t<SCALAR_T> GetLinearVelocity(const base_coordinate_s_t<SCALAR_T>& baseTwist) {
  return baseTwist.template tail<3>();
}

template <typename SCALAR_T>
/**
 * @brief Get Linear Acceleration.
 * @param [in] baseAcceleration: Base Acceleration.
 * @return Result value.
 */
vector3_s_t<SCALAR_T> GetLinearAcceleration(const base_coordinate_s_t<SCALAR_T>& baseAcceleration) {
  return baseAcceleration.template tail<3>();
}

template <typename SCALAR_T>
/**
 * @brief Get Joint Positions.
 * @param [in] comkinoState: Comkino State.
 * @return Result value.
 */
joint_coordinate_s_t<SCALAR_T> GetJointPositions(const Eigen::Matrix<SCALAR_T, -1, 1>& comkinoState) {
  return comkinoState.template segment<JOINT_COORDINATE_SIZE>(2 * BASE_COORDINATE_SIZE);
}

template <typename SCALAR_T>
/**
 * @brief Get Joint Positions.
 * @param [in] comkinoState: Comkino State.
 * @return Result value.
 */
joint_coordinate_s_t<SCALAR_T> GetJointPositions(const comkino_state_s_t<SCALAR_T>& comkinoState) {
  return comkinoState.template segment<JOINT_COORDINATE_SIZE>(2 * BASE_COORDINATE_SIZE);
}

template <typename SCALAR_T>
/**
 * @brief Get Joint Velocities.
 * @param [in] comkinoInput: Comkino Input.
 * @return Result value.
 */
joint_coordinate_s_t<SCALAR_T> GetJointVelocities(const Eigen::Matrix<SCALAR_T, -1, 1>& comkinoInput) {
  return comkinoInput.template segment<JOINT_COORDINATE_SIZE>(NUM_CONTACT_POINTS * 3);
}

template <typename SCALAR_T>
/**
 * @brief Get Joint Velocities.
 * @param [in] comkinoInput: Comkino Input.
 * @return Result value.
 */
joint_coordinate_s_t<SCALAR_T> GetJointVelocities(const comkino_input_s_t<SCALAR_T>& comkinoInput) {
  return comkinoInput.template segment<JOINT_COORDINATE_SIZE>(NUM_CONTACT_POINTS * 3);
}

template <typename SCALAR_T>
/**
 * @brief To Array.
 * @param [in] valuesAsVector: Values As Vector.
 * @return Result value.
 */
feet_array_t<vector3_s_t<SCALAR_T>> ToArray(const joint_coordinate_s_t<SCALAR_T>& valuesAsVector) {
  return {valuesAsVector.template segment<3>(0), valuesAsVector.template segment<3>(3), valuesAsVector.template segment<3>(6),
          valuesAsVector.template segment<3>(9)};
}

template <typename SCALAR_T>
/**
 * @brief From Array.
 * @param [in] valuesAsArray: Values As Array.
 * @return Result value.
 */
joint_coordinate_s_t<SCALAR_T> FromArray(const feet_array_t<vector3_s_t<SCALAR_T>>& valuesAsArray) {
  joint_coordinate_s_t<SCALAR_T> valuesAsVector;
  valuesAsVector << valuesAsArray[0], valuesAsArray[1], valuesAsArray[2], valuesAsArray[3];
  return valuesAsVector;
}

/**
 * Applies the function f to each leg individually. The inputs of f are to be passed per leg using feet_array<T>.
 *
 * Example usage:
 *  feet_array_t<scalar_t> numbers = {0.0, 1.0, 2.0, 3.0};
 *  feet_array_t<bool> flags = {true, false, true, false};
 *
 *  auto negativeIfTrue = [](scalar_t val, bool flag) { return flag ? val : -val; };
 *  const auto result = ApplyPerLeg(negativeIfTrue, numbers, flags); // = {-0.0, 1.0, -2.0, 3.0}
 */
template <typename Func, typename... T>
auto ApplyPerLeg(Func f, const feet_array_t<T>&... inputs) -> feet_array_t<decltype(f(inputs[0]...))> {
  return {f(inputs[0]...), f(inputs[1]...), f(inputs[2]...), f(inputs[3]...)};
}

/** Creates a feet array filled with constant values */
template <typename T>
/**
 * @brief Constant Feet Array.
 * @param [in] val: Val.
 * @return Result value.
 */
feet_array_t<T> ConstantFeetArray(const T& val) {
  return {val, val, val, val};
}

/**
 * @brief Count contact feet.
 * @param [in] contactFlags: Contact Flags.
 * @return Result value.
 */
inline int NumberOfClosedContacts(const contact_flag_t& contactFlags) {
  size_t numStanceLegs = 0;
  for (auto legInContact : contactFlags) {
    if (legInContact) {
      ++numStanceLegs;
    }
  }
  return numStanceLegs;
}

/**
 * @brief Number Of Open Contacts.
 * @param [in] contactFlags: Contact Flags.
 * @return Result value.
 */
inline int NumberOfOpenContacts(const contact_flag_t& contactFlags) {
  /**
   * @brief Number Of Closed Contacts.
   * @param [in] contactFlags: Contact Flags.
   * @return Result value.
   */
  return NUM_CONTACT_POINTS - NumberOfClosedContacts(contactFlags);
}

}  // end of namespace switched_model
