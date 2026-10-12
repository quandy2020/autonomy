//
// Created by rgrandia on 21.04.20.
//

#pragma once

/**
 * @file terrain_plane.hpp
 * @brief Planar terrain represented by a single coordinate frame.
 */


#include "bolza/switched_model/core/switched_model.hpp"

namespace switched_model {

/**
 * Planar terrain represented by a single coordinate frame.
 * - The terrain frame is located at positionInWorld.
 * - The surface normal points in positive z direction in the terrain frame.
 */
struct TerrainPlane {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  vector3_t positionInWorld;
  matrix3_t orientationWorldToTerrain;

  /**
   * @brief Constructs a TerrainPlane.
   */
  TerrainPlane() : positionInWorld(vector3_t::Zero()), orientationWorldToTerrain(matrix3_t::Identity()) {}
  TerrainPlane(vector3_t positionInWorld_, matrix3_t orientationWorldToTerrain_)
      /**
       * @brief Position In World.
       * @return Result value.
       */
      : positionInWorld(std::move(positionInWorld_)), orientationWorldToTerrain(std::move(orientationWorldToTerrain_)) {}

  explicit TerrainPlane(const Eigen::Isometry3d& transform)
      /**
       * @brief Position In World.
       * @return Result value.
       */
      : positionInWorld(transform.translation()), orientationWorldToTerrain(transform.linear().transpose()) {}
};

struct NormalAndPosition {
  vector3_t normal;
  vector3_t position;
};

/**
 * @brief Returns the surface normal = z-axis of the terrain, the unit vector is represented in the world frame.
 * @param [in] terrainPlane: Terrain Plane.
 * @return Result value.
 */
inline vector3_t SurfaceNormalInWorld(const TerrainPlane& terrainPlane) {
  /**
   * @brief Row.
   * @return Result value.
   */
  return terrainPlane.orientationWorldToTerrain.row(2).transpose();
}

/**
 * Constructs the x-y unit vectors for a given z-axis. The absolute orientation of the x-y vectors is unspecified.
 * @param [in] surfaceNormal: (z-axis) of the terrain.
 * @return 2x3 matrix forming the [x-axis; y-axis] of the terrain
 */
inline Eigen::Matrix<scalar_t, 2, 3> TangentialBasisFromSurfaceNormal(const vector3_t& surfaceNormal) {
  // Cross with any vector that is not equal to surfaceNormal
  Eigen::Vector3d yAxisInGlobal = surfaceNormal.cross(Eigen::Vector3d::UnitX());
  {  // Normalize the yAxis. Need to pick a different direction if z happened to intersect with unitX
    const auto ySquaredNorm = yAxisInGlobal.squaredNorm();
    const double crossTolerance = 1e-3;
    if (ySquaredNorm > crossTolerance) {
      yAxisInGlobal /= std::sqrt(ySquaredNorm);
    } else {
      // normal was almost equal to unitX. Pick the y-axis in a different way (approximately equal to unitY):
      yAxisInGlobal = surfaceNormal.cross(Eigen::Vector3d::UnitY().cross(surfaceNormal)).normalized();
    }
  }

  // Assumes the surface normal is normalized
  Eigen::Matrix<scalar_t, 2, 3> tangentBasis;
  /**
   * @brief Row.
   * @return Result value.
   */
  tangentBasis.row(0) = yAxisInGlobal.cross(surfaceNormal);
  tangentBasis.row(1) = yAxisInGlobal;
  return tangentBasis;
}

/**
 * @brief Constructs a rotation matrix from the specified surface normal (z-axis). The rotation around the surface normal is unspecified.
 * @param [in] surfaceNormal: Surface Normal.
 * @return Rotation matrix world to terrain, where the terrain frame has the z-axis aligned with the specified surface normal.
 */
inline matrix3_t OrientationWorldToTerrainFromSurfaceNormalInWorld(const vector3_t& surfaceNormal) {
  Eigen::Matrix<scalar_t, 2, 3> tangents = TangentialBasisFromSurfaceNormal(surfaceNormal);
  matrix3_t orientationWorldToTerrain;
  orientationWorldToTerrain.topRows(2) = tangents;
  /**
   * @brief Row.
   * @return Result value.
   */
  orientationWorldToTerrain.row(2) = surfaceNormal.transpose();
  return orientationWorldToTerrain;
}

/**
 * @brief Converts a 3D position in world frame to a 3D position in the terrain frame.
 * @param [in] positionWorld: Position World.
 * @param [in] terrainPlane: Terrain Plane.
 * @return Result value.
 */
inline vector3_t PositionInTerrainFrameFromPositionInWorld(const vector3_t& positionWorld, const TerrainPlane& terrainPlane) {
  return terrainPlane.orientationWorldToTerrain * (positionWorld - terrainPlane.positionInWorld);
}

/**
 * @brief Converts a 3D position in terrain frame to a 3D position in the world frame.
 * @param [in] positionInTerrain: Position In Terrain.
 * @param [in] terrainPlane: Terrain Plane.
 * @return Result value.
 */
inline vector3_t PositionInWorldFrameFromPositionInTerrain(const vector3_t& positionInTerrain, const TerrainPlane& terrainPlane) {
  return terrainPlane.orientationWorldToTerrain.transpose() * positionInTerrain + terrainPlane.positionInWorld;
}

/**
 * @brief Returns the orthogonal signed distance between the terrain a 3D point represented in world frame.
 * @param [in] positionWorld: Position World.
 * @param [in] terrainPlane: Terrain Plane.
 * @return Result value.
 */
inline scalar_t TerrainSignedDistanceFromPositionInWorld(const vector3_t& positionWorld, const TerrainPlane& terrainPlane) {
  const vector3_t surfaceNormal = SurfaceNormalInWorld(terrainPlane);
  /**
   * @brief Dot.
   * @param [in] positionInWorld: Position In World.
   * @return Result value.
   */
  return surfaceNormal.dot(positionWorld - terrainPlane.positionInWorld);
}

/**
 * @brief Returns the projection along gravity onto the terrain plane for a 2D position in world. The returned position is in world frame.
 * @param [in] positionXYWorld: Position XYWorld.
 * @param [in] terrainPlane: Terrain Plane.
 * @return Result value.
 */
inline vector3_t ProjectPositionInWorldOntoPlaneAlongGravity(const vector2_t& positionXYWorld, const TerrainPlane& terrainPlane) {
  // solve surfaceNormal.dot(projectedPosition - terrainPlane.positionInWorld) = 0
  // Distance = positionWorld.z() - projectedPosition.z()
  const vector3_t surfaceNormal = SurfaceNormalInWorld(terrainPlane);
  const scalar_t projectedPositionZ = (surfaceNormal.dot(terrainPlane.positionInWorld) - positionXYWorld.x() * surfaceNormal.x() -
                                       positionXYWorld.y() * surfaceNormal.y()) /
                                      /**
                                       * @brief Z.
                                       * @return Result value.
                                       */
                                      surfaceNormal.z();
  return {positionXYWorld.x(), positionXYWorld.y(), projectedPositionZ};
}

/**
 * @brief Returns the orthogonal projection onto the terrain plane for a 3D position in world. The returned position is in world frame.
 * @param [in] positionWorld: Position World.
 * @param [in] terrainPlane: Terrain Plane.
 * @return Result value.
 */
inline vector3_t ProjectPositionInWorldOntoPlane(const vector3_t& positionWorld, const TerrainPlane& terrainPlane) {
  const vector3_t surfaceNormal = SurfaceNormalInWorld(terrainPlane);
  return surfaceNormal.dot(terrainPlane.positionInWorld - positionWorld) * surfaceNormal + positionWorld;
}

/**
 * @brief Returns the projection along gravity onto the terrain plane for a 3D position in world. The returned position is in world frame.
 * @param [in] positionWorld: Position World.
 * @param [in] terrainPlane: Terrain Plane.
 * @return Result value.
 */
inline vector3_t ProjectPositionInWorldOntoPlaneAlongGravity(const vector3_t& positionWorld, const TerrainPlane& terrainPlane) {
  /**
   * @brief Project Position In World Onto Plane Along Gravity.
   * @param [in] terrainPlane: Terrain Plane.
   * @return Result value.
   */
  return ProjectPositionInWorldOntoPlaneAlongGravity(vector2_t{positionWorld.x(), positionWorld.y()}, terrainPlane);
}

/**
 * @brief Returns the projection along gravity onto the terrain orientation for a vector represented in world frame.
 *
 * This projection is such that the x-y components of the vector remain unchanged.
 *
 * The returned vector is still represented in the world frame.
 * @param [in] vectorInWorld: Vector In World.
 * @param [in] terrainPlane: Terrain Plane.
 * @return Result value.
 */
inline vector3_t ProjectVectorInWorldOntoPlaneAlongGravity(const vector3_t& vectorInWorld, const TerrainPlane& terrainPlane) {
  const vector3_t surfaceNormal = SurfaceNormalInWorld(terrainPlane);
  vector3_t projectedVector = vectorInWorld;
  // solve
  // 1. projectedVector.x() = vectorInWorld.x();
  // 2. projectedVector.y() = vectorInWorld.y();
  // 3. surfaceNormal.dot(projectedVector) = 0
  /**
   * @brief Z.
   * @return Result value.
   */
  projectedVector.z() = -(vectorInWorld.x() * surfaceNormal.x() + vectorInWorld.y() * surfaceNormal.y()) / surfaceNormal.z();
  return projectedVector;
}

/**
 * @brief Load Terrain Plane.
 * @param [in] filename: Filename.
 * @param [in] verbose: Verbose.
 * @return Result value.
 */
TerrainPlane LoadTerrainPlane(const std::string& filename, bool verbose);

}  // namespace switched_model
