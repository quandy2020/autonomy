//
// Created by rgrandia on 29.04.20.
//

#pragma once

/**
 * @file planar_terrain_model.hpp
 * @brief This class models the terrain as a single infinite plane.
 */


#include "bolza/switched_model/core/switched_model.hpp"
#include "bolza/switched_model/terrain/planar_signed_distance_field.hpp"
#include "bolza/switched_model/terrain/terrain_model.hpp"

namespace switched_model {

/**
 * This class models the terrain as a single infinite plane.
 */
class PlanarTerrainModel : public TerrainModel {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /**
   * @brief Constructs a PlanarTerrainModel.
   * @param [in] terrainPlane: Terrain Plane.
   */
  explicit PlanarTerrainModel(TerrainPlane terrainPlane);
  /**
   * @brief Destructor.
   */
  ~PlanarTerrainModel() override = default;

  /**
   * @brief Get Local Terrain At Position In World Along Gravity.
   * @param [in] positionInWorld: Position In World.
   * @param [in] penaltyFunction: Penalty Function.
   * @return Result value.
   */
  TerrainPlane GetLocalTerrainAtPositionInWorldAlongGravity(const vector3_t& positionInWorld,
                                                            std::function<scalar_t(const vector3_t&)> penaltyFunction) const override;

  /**
   * @brief Get Highest Obstacle Along Line.
   * @param [in] position1InWorld: Position1 In World.
   * @param [in] position2InWorld: Position2 In World.
   * @return Result value.
   */
  vector3_t GetHighestObstacleAlongLine(const vector3_t& position1InWorld, const vector3_t& position2InWorld) const override;

  /**
   * @brief Get Height Profile Along Line.
   * @param [in] position1InWorld: Position1 In World.
   * @param [in] position2InWorld: Position2 In World.
   * @return Result value.
   */
  std::vector<vector2_t> GetHeightProfileAlongLine(const vector3_t& position1InWorld, const vector3_t& position2InWorld) const override;

  /**
   * @brief Get Signed Distance Field.
   * @return Pointer; may be null.
   */
  const SignedDistanceField* GetSignedDistanceField() const override { return &sdf_; }

 private:
  TerrainPlane terrainPlane_;
  PlanarSignedDistanceField sdf_;
};

}  // namespace switched_model
