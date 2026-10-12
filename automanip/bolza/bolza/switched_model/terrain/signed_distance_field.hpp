//
// Created by rgrandia on 14.08.20.
//

#pragma once

/**
 * @file signed_distance_field.hpp
 * @brief This abstract class defines the interface for a signed distance field.
 */


#include "bolza/switched_model/core/switched_model.hpp"

namespace switched_model {

/**
 * This abstract class defines the interface for a signed distance field.
 */
class SignedDistanceField {
 public:
  /**
   * @brief Constructs a SignedDistanceField.
   */
  SignedDistanceField() = default;
  /**
   * @brief Destructor.
   */
  virtual ~SignedDistanceField() = default;
  /**
   * @brief Constructs a SignedDistanceField.
   */
  SignedDistanceField(const SignedDistanceField&) = delete;
  SignedDistanceField& operator=(const SignedDistanceField&) = delete;

  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  virtual SignedDistanceField* Clone() const = 0;
  /**
   * @brief Value.
   * @param [in] position: Position.
   * @return Result value.
   */
  virtual scalar_t Value(const vector3_t& position) const = 0;
  /**
   * @brief Derivative.
   * @param [in] position: Position.
   * @return Result value.
   */
  virtual Eigen::Vector3d Derivative(const vector3_t& position) const = 0;
  /**
   * @brief Value And Derivative.
   * @param [in] position: Position.
   * @return Result value.
   */
  virtual std::pair<scalar_t, vector3_t> ValueAndDerivative(const vector3_t& position) const = 0;
};

}  // namespace switched_model