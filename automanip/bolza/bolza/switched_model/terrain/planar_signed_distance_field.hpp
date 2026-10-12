#pragma once

/**
 * @file planar_signed_distance_field.hpp
 * @brief Implements a flat terrain signed distance field.
 */


#include "bolza/switched_model/core/switched_model.hpp"
#include "bolza/switched_model/terrain/signed_distance_field.hpp"
#include "bolza/switched_model/terrain/terrain_model.hpp"

namespace switched_model {

/**
 * Implements a flat terrain signed distance field
 */
class PlanarSignedDistanceField : public SignedDistanceField {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Constructs a PlanarSignedDistanceField.
   * @param [in] terrainPlane: Terrain Plane.
   */
  explicit PlanarSignedDistanceField(TerrainPlane terrainPlane);

  /**
   * @brief Destructor.
   */
  ~PlanarSignedDistanceField() override = default;
  /**
   * @brief Clone.
   * @param [in] this: This.
   * @return Pointer; may be null.
   */
  PlanarSignedDistanceField* Clone() const override { return new PlanarSignedDistanceField(*this); };

  /**
   * @brief Value.
   * @param [in] position: Position.
   * @return Result value.
   */
  scalar_t Value(const vector3_t& position) const override;

  /**
   * @brief Derivative.
   * @param [in] position: Position.
   * @return Result value.
   */
  vector3_t Derivative(const vector3_t& position) const override;

  /**
   * @brief Value And Derivative.
   * @param [in] position: Position.
   * @return Result value.
   */
  std::pair<scalar_t, vector3_t> ValueAndDerivative(const vector3_t& position) const override;

 protected:
  /**
   * @brief Constructs a PlanarSignedDistanceField.
   * @param [in] other: Other.
   */
  PlanarSignedDistanceField(const PlanarSignedDistanceField& other);

 private:
  TerrainPlane terrainPlane_;
};

}  // namespace switched_model
