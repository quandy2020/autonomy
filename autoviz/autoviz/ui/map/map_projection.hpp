/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QPointF>
#include <QRect>
#include <QSize>

#include <cstddef>

namespace autoviz {
namespace map {

struct MapTileCoord {
  int z = 0;
  int x = 0;
  int y = 0;

  bool operator==(const MapTileCoord& other) const {
    return z == other.z && x == other.x && y == other.y;
  }
};

struct MapTileCoordHash {
  std::size_t operator()(const MapTileCoord& key) const noexcept {
    return static_cast<std::size_t>(key.z) * 73856093u ^
           static_cast<std::size_t>(key.x) * 19349663u ^
           static_cast<std::size_t>(key.y) * 83492791u;
  }
};

inline std::size_t qHash(const MapTileCoord& key, std::size_t seed = 0) noexcept {
  return MapTileCoordHash{}(key) ^ seed;
}

/** Web Mercator projection helpers for slippy-map tile rendering. */
class MapProjection {
 public:
  static double ClampLatitude(double latitude);
  static double LongitudeToWorldX(double longitude);
  static double LatitudeToWorldY(double latitude);
  static double WorldXToLongitude(double world_x);
  static double WorldYToLatitude(double world_y);

  static MapTileCoord LatLonToTile(double latitude, double longitude, int zoom);
  static QPointF LatLonToWorldPixel(double latitude, double longitude, int zoom);
  static QPointF WorldPixelToLatLon(const QPointF& world_pixel, int zoom);

  static int TileCount(int zoom);
  static QRect VisibleTileRange(const QPointF& center_world_pixel,
                                const QSize& viewport_size, int zoom,
                                int margin_tiles = 1);
};

}  // namespace map
}  // namespace autoviz
