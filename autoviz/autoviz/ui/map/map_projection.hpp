/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file map_projection.hpp
 * @brief Web Mercator helpers for slippy-map tile coordinates and pixels.
 *
 * Converts WGS84 lat/lon ↔ world pixels / tile indices used by
 * @ref MapViewportWidget and @ref MapTileCache.
 *
 * @see MapTileCoord
 * @see MapViewportWidget
 */

#pragma once

#include <QPointF>
#include <QRect>
#include <QSize>

#include <cstddef>

namespace autoviz {
namespace map {

/**
 * @struct MapTileCoord
 * @brief Slippy-map tile index at a given zoom (@c z / @c x / @c y).
 */
struct MapTileCoord {
  int z = 0;  /**< Zoom level. */
  int x = 0;  /**< Tile column. */
  int y = 0;  /**< Tile row. */

  /**
   * @brief Equality for use as a hash-map key.
   *
   * @param other Tile to compare.
   * @return @c true when z/x/y all match.
   */
  bool operator==(const MapTileCoord& other) const {
    return z == other.z && x == other.x && y == other.y;
  }
};

/**
 * @struct MapTileCoordHash
 * @brief STL-style hash functor for @ref MapTileCoord.
 */
struct MapTileCoordHash {
  /**
   * @brief Mixes z/x/y into a size_t hash.
   *
   * @param key Tile coordinate.
   * @return Hash value.
   */
  std::size_t operator()(const MapTileCoord& key) const noexcept {
    return static_cast<std::size_t>(key.z) * 73856093u ^
           static_cast<std::size_t>(key.x) * 19349663u ^
           static_cast<std::size_t>(key.y) * 83492791u;
  }
};

/**
 * @brief Qt hash overload so @ref MapTileCoord works in @c QHash / @c QSet.
 *
 * @param key Tile coordinate.
 * @param seed Optional Qt hash seed.
 * @return Combined hash.
 */
inline std::size_t qHash(const MapTileCoord& key, std::size_t seed = 0) noexcept {
  return MapTileCoordHash{}(key) ^ seed;
}

/**
 * @class MapProjection
 * @brief Web Mercator projection helpers for slippy-map tile rendering.
 *
 * All methods are static. Latitude is clamped to the Web Mercator valid range
 * (~±85.0511°) before projection.
 */
class MapProjection {
 public:
  /**
   * @brief Clamps latitude to the Web Mercator valid range.
   *
   * @param latitude Latitude in degrees.
   * @return Clamped latitude.
   */
  static double ClampLatitude(double latitude);

  /**
   * @brief Maps longitude to continuous world X at zoom-independent units.
   *
   * @param longitude Longitude in degrees.
   * @return World X in \[0, 1) tile-space at zoom 0 scale.
   */
  static double LongitudeToWorldX(double longitude);

  /**
   * @brief Maps latitude to continuous world Y (Web Mercator).
   *
   * @param latitude Latitude in degrees.
   * @return World Y in \[0, 1) tile-space at zoom 0 scale.
   */
  static double LatitudeToWorldY(double latitude);

  /**
   * @brief Inverse of @ref LongitudeToWorldX().
   *
   * @param world_x World X.
   * @return Longitude in degrees.
   */
  static double WorldXToLongitude(double world_x);

  /**
   * @brief Inverse of @ref LatitudeToWorldY().
   *
   * @param world_y World Y.
   * @return Latitude in degrees.
   */
  static double WorldYToLatitude(double world_y);

  /**
   * @brief Tile containing the given lat/lon at @p zoom.
   *
   * @param latitude Latitude (degrees).
   * @param longitude Longitude (degrees).
   * @param zoom Integer zoom level.
   * @return Tile coordinate.
   */
  static MapTileCoord LatLonToTile(double latitude, double longitude, int zoom);

  /**
   * @brief Converts lat/lon to world-pixel coordinates at @p zoom.
   *
   * @param latitude Latitude (degrees).
   * @param longitude Longitude (degrees).
   * @param zoom Integer zoom level.
   * @return Pixel position in the global tile mosaic (origin top-left).
   */
  static QPointF LatLonToWorldPixel(double latitude, double longitude, int zoom);

  /**
   * @brief Inverse of @ref LatLonToWorldPixel().
   *
   * @param world_pixel World-pixel position.
   * @param zoom Integer zoom level.
   * @return @c QPointF as (latitude, longitude) in degrees.
   */
  static QPointF WorldPixelToLatLon(const QPointF& world_pixel, int zoom);

  /**
   * @brief Number of tiles along one axis at @p zoom (@c 2^zoom).
   *
   * @param zoom Integer zoom level.
   * @return Tile count per axis.
   */
  static int TileCount(int zoom);

  /**
   * @brief Inclusive tile index rectangle covering the viewport plus margin.
   *
   * @param center_world_pixel Viewport center in world pixels.
   * @param viewport_size Widget size in pixels.
   * @param zoom Integer zoom level.
   * @param margin_tiles Extra tiles beyond the visible edges (default 1).
   * @return Tile range as @c QRect(@c x_min, @c y_min, @c width, @c height).
   */
  static QRect VisibleTileRange(const QPointF& center_world_pixel,
                                const QSize& viewport_size, int zoom,
                                int margin_tiles = 1);
};

}  // namespace map
}  // namespace autoviz
