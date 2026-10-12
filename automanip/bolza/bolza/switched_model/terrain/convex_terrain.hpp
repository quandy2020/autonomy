//
// Created by rgrandia on 23.06.20.
//

#pragma once

/**
 * @file convex_terrain.hpp
 * @brief Projects a 2D point into boundary of a 2D convex polygon.
 */


#include <limits>
#include <utility>

#include "bolza/switched_model/core/switched_model.hpp"

#include "bolza/switched_model/terrain/terrain_plane.hpp"

namespace switched_model {

struct ConvexTerrain {
  /// Plane coordinate defining the origin of the terrain
  TerrainPlane plane;

  /// Boundary points x-y in the terrain frame, points are order counter-clockwise. The last point is connected to the first point
  std::vector<vector2_t> boundary;
};

/**
 * @brief Get Next Vertex.
 * @param [in] i: I.
 * @param [in] N: N.
 * @return Result value.
 */
inline int GetNextVertex(int i, size_t N) {
  int next = i + 1;
  return (next < N) ? next : 0;  // next point with wrap around
}

/**
 * @brief Get Previous Vertex.
 * @param [in] i: I.
 * @param [in] N: N.
 * @return Result value.
 */
inline int GetPreviousVertex(int i, size_t N) {
  return (i > 0) ? (i - 1) : (N - 1);  // previous point with wrap around
}

/**
 * Projects a 2D point into boundary of a 2D convex polygon.
 * @param [in] boundary: The vertices of the polygon in clockwise or counter-clockwise order.
 * @param [in] p: The 2D point.
 * @return A pair of signed squared distance to the boundary (negative inside, positive outside) and the projected point.
 */
inline std::pair<scalar_t, vector2_t> ProjectToConvex2dPolygonBoundary(const std::vector<vector2_t>& boundary, const vector2_t& p) {
  vector2_t image = p;
  scalar_t dist2 = std::numeric_limits<scalar_t>::max();
  auto saveIfCloser = [&p, &dist2, &image](const vector2_t& q) {
    const scalar_t newDist2 = (p - q).squaredNorm();
    if (newDist2 < dist2) {
      dist2 = newDist2;
      image = q;
    }
  };

  bool isInside = true;
  for (int i = 0; i < boundary.size(); i++) {
    const auto& p1 = boundary[i];
    const auto& p2 = boundary[GetNextVertex(i, boundary.size())];

    const vector2_t p12 = p2 - p1;
    const scalar_t r = p12.dot(p - p1) / p12.squaredNorm();

    if (r > 1.0) {
      /**
       * @brief Constructs a SaveIfCloser.
       * @param [in] p2: P2.
       */
      saveIfCloser(p2);
    } else if (r < 0.0) {
      /**
       * @brief Constructs a SaveIfCloser.
       * @param [in] p1: P1.
       */
      saveIfCloser(p1);
      isInside = false;  // the point is outside since the angle is obtuse
    } else {
      const vector2_t q = p1 + r * p12;
      /**
       * @brief Constructs a SaveIfCloser.
       * @param [in] q: Q.
       */
      saveIfCloser(q);
    }
  }  // end of i loop

  return {(isInside ? -dist2 : dist2), image};
}

/**
 * Projects a 3D point onto a 3D convex polygon.
 * @param [in] convexTerrain: The 3D convex polygon.
 * @param [in] p: The 3D point.
 * @return The projected point.
 */
inline vector3_t ProjectToConvex3dPolygon(const ConvexTerrain& convexTerrain, const vector3_t& p) {
  const vector3_t local_p = PositionInTerrainFrameFromPositionInWorld(p, convexTerrain.plane);
  /**
   * @brief Local 2d p.
   * @return Result value.
   */
  const vector2_t local_2d_p(local_p.x(), local_p.y());

  const auto distance2ImagePair = ProjectToConvex2dPolygonBoundary(convexTerrain.boundary, local_2d_p);

  vector3_t local_q;
  if (distance2ImagePair.first <= 0.0) {
    // the 2d local point is inside polygon
    local_q << local_2d_p, 0.0;
  } else {
    // the 2d local point is outside polygon
    local_q << distance2ImagePair.second, 0.0;
  }

  /**
   * @brief Position In World Frame From Position In Terrain.
   * @param [in] local_q: Local q.
   * @param [in] plane: Plane.
   * @return Result value.
   */
  return PositionInWorldFrameFromPositionInTerrain(local_q, convexTerrain.plane);
}

}  // namespace switched_model
