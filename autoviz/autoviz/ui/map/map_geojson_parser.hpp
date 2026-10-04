/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file map_geojson_parser.hpp
 * @brief GeoJSON → geographic feature structs for the Map panel.
 *
 * Defines point / line / polygon primitives and @ref MapIngestResult used by
 * @ref MapGeoJsonParser and @ref MapMessageIngest before insertion into
 * @ref MapLayerStore.
 *
 * @see MapLayerStore
 * @see MapMessageIngest
 */

#pragma once

#include <QColor>
#include <QPointF>
#include <QString>
#include <QVector>

#include "autoviz/ui/map/map_types.hpp"

namespace autoviz {
namespace map {

/**
 * @struct MapGeoPoint
 * @brief A timestamped geographic point with optional kinematics and style.
 *
 * @c QPointF overlays elsewhere store @c (longitude, latitude); this struct
 * keeps lat/lon as named fields for clarity at the ingest boundary.
 */
struct MapGeoPoint {
  double latitude = 0.0;   /**< WGS84 latitude (degrees). */
  double longitude = 0.0;  /**< WGS84 longitude (degrees). */
  double heading_deg = qQNaN();  /**< Heading in degrees; NaN if unknown. */
  double velocity_east_mps = qQNaN();   /**< East velocity (m/s); NaN if unknown. */
  double velocity_north_mps = qQNaN();  /**< North velocity (m/s); NaN if unknown. */
  quint64 timestamp_ns = 0;  /**< Source timestamp (nanoseconds). */
  QString label;             /**< Optional text label. */
  QColor color;              /**< Optional per-point color override. */
  QVector<MapAttribute> attributes; /**< Feature properties, excluding paint keys. */
};

/**
 * @struct MapGeoLine
 * @brief Polyline in geographic coordinates.
 *
 * Each @c QPointF in @c lat_lon_points is @c (longitude, latitude).
 */
struct MapGeoLine {
  QVector<QPointF> lat_lon_points;  /**< Vertices as (lon, lat). */
  QColor color = QColor(80, 140, 255);  /**< Stroke color. */
  float width = 2.0f;  /**< Stroke width in screen pixels. */
  QString label;  /**< Feature name from GeoJSON properties, if any. */
  QVector<MapAttribute> attributes; /**< Feature properties, excluding paint keys. */
};

/**
 * @struct MapGeoPolygon
 * @brief Filled polygon with a single outer ring (lon, lat) vertices.
 */
struct MapGeoPolygon {
  QVector<QPointF> outer_ring;  /**< Outer ring as (lon, lat); closed implicitly. */
  QColor fill_color = QColor(80, 140, 255, 80);  /**< Fill color (with alpha). */
  QColor outline_color = QColor(80, 140, 255);   /**< Outline stroke color. */
  float outline_width = 2.0f;  /**< Outline width in screen pixels. */
  QString label;  /**< Feature name from GeoJSON properties, if any. */
  QVector<MapAttribute> attributes; /**< Feature properties, excluding paint keys. */
};

/**
 * @struct MapIngestResult
 * @brief Batch of geographic features produced by one message parse.
 *
 * When @c append_trail is @c true, points are appended to the channel trail
 * in @ref MapLayerStore rather than replacing the snapshot set.
 */
struct MapIngestResult {
  QVector<MapGeoPoint> points;      /**< Point features. */
  QVector<MapGeoLine> lines;        /**< Line features. */
  QVector<MapGeoPolygon> polygons;  /**< Polygon features. */
  bool append_trail = false;       /**< Append points to trail vs replace snapshot. */
};

/**
 * @class MapGeoJsonParser
 * @brief Stateless GeoJSON text parser producing @ref MapIngestResult.
 */
class MapGeoJsonParser {
 public:
  /**
   * @brief Parses a GeoJSON document into map features.
   *
   * @param geojson_text UTF-8 GeoJSON Feature / FeatureCollection text.
   * @param error Optional out-parameter for a human-readable error string.
   * @return Ingest result (empty on failure; @p error set when provided).
   */
  static MapIngestResult Parse(const QString& geojson_text, QString* error);
};

}  // namespace map
}  // namespace autoviz
