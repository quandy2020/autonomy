//
// Created by rgrandia on 21.04.20.
//

#pragma once

/**
 * @file terrain_model.hpp
 * @brief This abstract class defines the interface for terrain models.
 */


#include "bolza/switched_model/core/switched_model.hpp"
#include "bolza/switched_model/terrain/convex_terrain.hpp"
#include "bolza/switched_model/terrain/signed_distance_field.hpp"
#include "bolza/switched_model/terrain/terrain_plane.hpp"

namespace switched_model {

/**
 * This abstract class defines the interface for terrain models.
 */
class TerrainModel {
 public:
  /**
   * @brief Constructs a TerrainModel.
   */
  TerrainModel() = default;
  /**
   * @brief Destructor.
   */
  virtual ~TerrainModel() = default;
  /**
   * @brief Constructs a TerrainModel.
   */
  TerrainModel(const TerrainModel&) = delete;
  TerrainModel& operator=(const TerrainModel&) = delete;

  /** Returns a linear approximation of the terrain at the query point projected along gravity onto the terrain  */
  // TODO: change name and remove "alongGravity"
  /**
   * @brief Get Local Terrain At Position In World Along Gravity.
   * @param [in] positionInWorld: Position In World.
   * @return Result value.
   */
  TerrainPlane GetLocalTerrainAtPositionInWorldAlongGravity(const vector3_t& positionInWorld) const {
    /**
     * @brief Get Local Terrain At Position In World Along Gravity.
     * @param [in] positionInWorld: Position In World.
     * @return Result value.
     */
    return GetLocalTerrainAtPositionInWorldAlongGravity(positionInWorld, [](const vector3_t&) { return 0.0; });
  }

  /// Penalty function needs to return values >= 0
  /**
   * @brief Get Local Terrain At Position In World Along Gravity.
   * @param [in] positionInWorld: Position In World.
   * @param [in] penaltyFunction: Penalty Function.
   * @return Result value.
   */
  virtual TerrainPlane GetLocalTerrainAtPositionInWorldAlongGravity(const vector3_t& positionInWorld,
                                                                    std::function<scalar_t(const vector3_t&)> penaltyFunction) const = 0;

  /**
   * @brief Get Convex Terrain At Position In World.
   * @param [in] positionInWorld: Position In World.
   * @return Result value.
   */
  ConvexTerrain GetConvexTerrainAtPositionInWorld(const vector3_t& positionInWorld) const {
    /**
     * @brief Get Convex Terrain At Position In World.
     * @param [in] positionInWorld: Position In World.
     * @return Result value.
     */
    return GetConvexTerrainAtPositionInWorld(positionInWorld, [](const vector3_t&) { return 0.0; });
  }

  /// Penalty function needs to return values >= 0
  /**
   * @brief Get Convex Terrain At Position In World.
   * @param [in] positionInWorld: Position In World.
   * @param [in] penaltyFunction: Penalty Function.
   * @return Result value.
   */
  virtual ConvexTerrain GetConvexTerrainAtPositionInWorld(const vector3_t& positionInWorld,
                                                          std::function<scalar_t(const vector3_t&)> penaltyFunction) const {
    return {GetLocalTerrainAtPositionInWorldAlongGravity(positionInWorld, std::move(penaltyFunction)), {}};
  }

  /**
   * @brief Returns the signed distance field for this terrain if one is available.
   * @return Pointer; may be null.
   */
  virtual const SignedDistanceField* GetSignedDistanceField() const { return nullptr; }

  /**
   * @brief Get Highest Obstacle Along Line.
   * @param [in] position1InWorld: Position1 In World.
   * @param [in] position2InWorld: Position2 In World.
   * @return Result value.
   */
  virtual vector3_t GetHighestObstacleAlongLine(const vector3_t& position1InWorld, const vector3_t& position2InWorld) const = 0;

  /**
   * @brief Returns the height profiles between two points in world frame. Provided as a set of points {alpha, height}, where alpha in [0, 1] is.
   *
   * the progress along the line. position1InWorld -> alpha = 0, position2InWorld -> alpha = 1.
   *
   * Height is the absolute height in world frame.
   * @param [in] position1InWorld: Position1 In World.
   * @param [in] position2InWorld: Position2 In World.
   * @return Result value.
   */
  virtual std::vector<vector2_t> GetHeightProfileAlongLine(const vector3_t& position1InWorld, const vector3_t& position2InWorld) const = 0;
};

}  // namespace switched_model
