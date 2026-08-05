/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/map/map_projection.hpp"

#include <QtMath>

#include <algorithm>
#include <cmath>

namespace autoviz {
namespace map {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kOriginShift = 20037508.342789244;
constexpr int kTilePixelSize = 256;

double DegToRad(double degrees) { return degrees * kPi / 180.0; }

double RadToDeg(double radians) { return radians * 180.0 / kPi; }

}  // namespace

double MapProjection::ClampLatitude(double latitude) {
  return std::clamp(latitude, -85.05112878, 85.05112878);
}

double MapProjection::LongitudeToWorldX(double longitude) {
  return DegToRad(longitude) * kOriginShift / kPi;
}

double MapProjection::LatitudeToWorldY(double latitude) {
  const double lat = ClampLatitude(latitude);
  const double sin_lat = std::sin(DegToRad(lat));
  return kOriginShift * std::log((1.0 + sin_lat) / (1.0 - sin_lat)) / kPi;
}

double MapProjection::WorldXToLongitude(double world_x) {
  return RadToDeg(world_x * kPi / kOriginShift);
}

double MapProjection::WorldYToLatitude(double world_y) {
  const double exp_term = std::exp(world_y * kPi / kOriginShift);
  return RadToDeg(2.0 * std::atan(exp_term) - kPi / 2.0);
}

MapTileCoord MapProjection::LatLonToTile(double latitude, double longitude,
                                       int zoom) {
  const int tile_count = TileCount(zoom);
  const double lat = ClampLatitude(latitude);
  const double n = std::pow(2.0, zoom);
  const int x = static_cast<int>(std::floor((longitude + 180.0) / 360.0 * n));
  const int y = static_cast<int>(std::floor(
      (1.0 - std::log(std::tan(DegToRad(lat)) + 1.0 / std::cos(DegToRad(lat))) /
                    kPi) /
      2.0 * n));
  MapTileCoord tile;
  tile.z = zoom;
  tile.x = std::clamp(x, 0, tile_count - 1);
  tile.y = std::clamp(y, 0, tile_count - 1);
  return tile;
}

QPointF MapProjection::LatLonToWorldPixel(double latitude, double longitude,
                                          int zoom) {
  const double scale = static_cast<double>(kTilePixelSize) *
                       static_cast<double>(TileCount(zoom)) / (2.0 * kOriginShift);
  const double world_x = LongitudeToWorldX(longitude);
  const double world_y = LatitudeToWorldY(latitude);
  return QPointF(world_x * scale, world_y * scale);
}

QPointF MapProjection::WorldPixelToLatLon(const QPointF& world_pixel, int zoom) {
  const double scale = static_cast<double>(kTilePixelSize) *
                       static_cast<double>(TileCount(zoom)) / (2.0 * kOriginShift);
  const double world_x = world_pixel.x() / scale;
  const double world_y = world_pixel.y() / scale;
  return QPointF(WorldYToLatitude(world_y), WorldXToLongitude(world_x));
}

int MapProjection::TileCount(int zoom) {
  return 1 << std::clamp(zoom, 0, 22);
}

QRect MapProjection::VisibleTileRange(const QPointF& center_world_pixel,
                                      const QSize& viewport_size, int zoom,
                                      int margin_tiles) {
  const int tile_count = TileCount(zoom);
  const double half_w = viewport_size.width() * 0.5;
  const double half_h = viewport_size.height() * 0.5;
  const int min_x = static_cast<int>(
      std::floor((center_world_pixel.x() - half_w) / kTilePixelSize)) -
      margin_tiles;
  const int max_x = static_cast<int>(
      std::floor((center_world_pixel.x() + half_w) / kTilePixelSize)) +
      margin_tiles;
  const int min_y = static_cast<int>(
      std::floor((center_world_pixel.y() - half_h) / kTilePixelSize)) -
      margin_tiles;
  const int max_y = static_cast<int>(
      std::floor((center_world_pixel.y() + half_h) / kTilePixelSize)) +
      margin_tiles;
  return QRect(std::clamp(min_x, 0, tile_count - 1),
               std::clamp(min_y, 0, tile_count - 1),
               std::clamp(max_x, 0, tile_count - 1) -
                   std::clamp(min_x, 0, tile_count - 1) + 1,
               std::clamp(max_y, 0, tile_count - 1) -
                   std::clamp(min_y, 0, tile_count - 1) + 1);
}

}  // namespace map
}  // namespace autoviz
