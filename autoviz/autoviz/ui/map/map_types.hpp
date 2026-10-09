/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file map_types.hpp
 * @brief Shared enums, layer configs, and helpers for the Map panel.
 *
 * Defines basemap presets, point styles, time-range modes, and
 * @ref MapPanelConfig consumed by @ref MapPanel / @ref MapSettingsWidget /
 * @ref MapViewportWidget.
 *
 * @see MapPanelConfig
 * @see DefaultMapPanelConfig()
 */

#pragma once

#include <QColor>
#include <QString>
#include <QVector>

namespace autoviz {
namespace map {

/**
 * @enum MapBaseLayer
 * @brief Built-in basemap tile provider presets (or custom URL).
 */
enum class MapBaseLayer {
  kStreet = 0,         /**< Street basemap (CARTO Voyager / OSM data). */
  kSatellite = 1,      /**< Esri World Imagery. */
  kShadedRelief = 2,   /**< Terrain / shaded relief (Esri World Terrain). */
  kCustom = 3,         /**< Use @ref MapPanelConfig::custom_tile_url. */
  kEsriStreet = 4,     /**< Esri World Street. */
  kEsriTerrain = 5,    /**< Esri World Terrain. */
  kCartoVoyager = 6,   /**< CARTO Voyager. */
  kJapanStandard = 7,  /**< Japan GSI standard map. */
  kAmapStreet = 8,     /**< Amap / 高德矢量路网. */
  kAmapSatellite = 9,  /**< Amap / 高德卫星影像. */
  kBingRoad = 10,      /**< Bing Maps road. */
  kBingAerial = 11,    /**< Bing Maps aerial. */
};

/**
 * @enum MapEditTool
 * @brief What a left click on the map does after a pan gesture is ruled out.
 */
enum class MapEditTool {
  kPan = 0,       /**< Select and measure. Drag still pans. */
  kWaypoint = 1,  /**< Append a mission waypoint. */
  kGeofence = 2,  /**< Append a geofence vertex. */
  kRally = 3,     /**< Append a rally point. */
};

/**
 * @struct MapPlanPoint
 * @brief One latitude/longitude vertex in a mission plan.
 */
struct MapPlanPoint {
  double latitude = 0;
  double longitude = 0;
};

/**
 * @enum MapPointStyle
 * @brief Glyph used when drawing topic-layer points.
 */
enum class MapPointStyle {
  kDot = 0,     /**< Filled circle. */
  kArrow = 1,   /**< Heading arrow. */
  kDiamond = 2, /**< Diamond marker. */
  kSquare = 3,  /**< Square marker. */
  kCross = 4,   /**< Cross / plus marker. */
};

/**
 * @enum MapDistanceUnit
 * @brief Ground distance unit used by the scale bar.
 */
enum class MapDistanceUnit {
  kMeters = 0, /**< Meters / kilometers (QGC metric). */
  kFeet = 1,   /**< Feet / miles. */
};

/**
 * @enum MapTimeRange
 * @brief Which historical points from a channel trail are drawn.
 */
enum class MapTimeRange {
  kLatest = 0,       /**< Only the most recent point. */
  kLastNSeconds = 1, /**< Points within @c time_range_seconds of now. */
  kAll = 2,          /**< Entire retained trail. */
};

/**
 * @struct MapOverlayLayerConfig
 * @brief Optional tile overlay drawn above the basemap.
 */
struct MapOverlayLayerConfig {
  QString name;               /**< Display name in the settings list. */
  QString tile_url_template;  /**< URL template with @c {z}/{x}/{y}. */
  double opacity = 0.5;       /**< Overlay opacity in \[0, 1\]. */
  bool enabled = true;        /**< When @c false, overlay is skipped. */
};

/**
 * @struct MapAttribute
 * @brief One feature property shown in the map inspector.
 */
struct MapAttribute {
  QString key;
  QString value;
};

/**
 * @struct MapSelectionInfo
 * @brief The feature or plan vertex currently shown in the attribute inspector.
 */
struct MapSelectionInfo {
  bool valid = false;
  QString title;
  QString kind;
  QString layer;
  double latitude = 0.0;
  double longitude = 0.0;
  int plan_kind = -1;   /**< 0 waypoint, 1 geofence, 2 rally; −1 if not a plan vertex. */
  int plan_index = -1;  /**< Index inside that plan list. */
  QVector<MapAttribute> attributes;
};

/**
 * @struct MapGeoJsonSource
 * @brief A GeoJSON file kept with the map document.
 */
struct MapGeoJsonSource {
  QString path;             /**< Absolute filesystem path. */
  bool visible = true;      /**< When @c false, the layer is not drawn or picked. */
};

/**
 * @struct MapTopicLayerConfig
 * @brief Style and filter options for one subscribed geo channel.
 */
struct MapTopicLayerConfig {
  QString channel;  /**< Source channel name. */
  MapPointStyle point_style = MapPointStyle::kDot;  /**< Point glyph. */
  bool show_heading = true;   /**< Draw heading when available. */
  bool show_velocity = false; /**< Draw velocity vector when available. */
  double point_size = 8.0;    /**< Marker size in screen pixels. */
  MapTimeRange time_range = MapTimeRange::kLatest;  /**< Trail filter mode. */
  double time_range_seconds = 30.0;  /**< Window for @ref MapTimeRange::kLastNSeconds. */
  double layer_opacity = 1.0; /**< Layer opacity in \[0, 1\]. */
  QColor color;               /**< Default layer color. */
  bool enabled = true;        /**< When @c false, layer is not drawn. */
};

/**
 * @struct MapPanelConfig
 * @brief Full persisted configuration for an @ref MapPanel instance.
 */
struct MapPanelConfig {
  QString title;  /**< Panel / dock title. */
  MapBaseLayer base_layer = MapBaseLayer::kStreet;  /**< Basemap preset. */
  QString custom_tile_url;  /**< Custom URL when @ref MapBaseLayer::kCustom. */
  QString follow_channel;   /**< Channel to follow with the camera (empty = none). */
  QString gcs_channel;      /**< Ground-station position channel (empty = none). */
  MapDistanceUnit distance_unit = MapDistanceUnit::kMeters; /**< Scale-bar units. */
  double center_latitude = 39.9042;   /**< Initial / saved center latitude. */
  double center_longitude = 116.4074; /**< Initial / saved center longitude. */
  double zoom = 14.0;                 /**< Slippy-map zoom level. */
  QVector<MapOverlayLayerConfig> overlay_layers;  /**< Tile overlays. */
  QVector<MapTopicLayerConfig> topic_layers;       /**< Geo topic layers. */
  QVector<MapGeoJsonSource> geojson_sources;       /**< Loaded GeoJSON files. */
  MapEditTool edit_tool = MapEditTool::kPan;       /**< Click tool. */
  QVector<MapPlanPoint> waypoints;                 /**< Mission path. */
  QVector<MapPlanPoint> geofence;                  /**< Inclusion polygon. */
  QVector<MapPlanPoint> rally_points;              /**< Rally markers. */
  double survey_spacing_m = 40.0;                  /**< Survey pass spacing. */
  double corridor_width_m = 50.0;                  /**< Corridor full width. */
  double structure_radius_m = 80.0;                /**< Structure-scan radius. */
};

/**
 * @brief Returns a default map panel config (Beijing center, street basemap).
 *
 * @return Sensible defaults for a new Map panel.
 */
MapPanelConfig DefaultMapPanelConfig();

/**
 * @brief Resolves the tile URL template for a basemap preset.
 *
 * @param layer Basemap enum value.
 * @param custom_url Custom template used when @p layer is @ref MapBaseLayer::kCustom.
 * @return URL template with @c {z}/{x}/{y}, optional @c {s} subdomain, and
 *         optional Bing @c {quadkey}.
 */
QString BaseLayerTileUrlTemplate(MapBaseLayer layer, const QString& custom_url);

/**
 * @brief Highest slippy zoom the basemap actually serves.
 *
 * Sampling above this upscales a parent tile and softens street labels.
 *
 * @param layer Basemap enum value.
 * @return Native maximum zoom.
 */
int BaseLayerMaxNativeZoom(MapBaseLayer layer);

/**
 * @brief Human-readable label for a basemap preset (settings combo).
 *
 * @param layer Basemap enum value.
 * @return Localized display string.
 */
QString BaseLayerLabel(MapBaseLayer layer);

/**
 * @brief Human-readable label for a point style.
 *
 * @param style Point style enum value.
 * @return Localized display string.
 */
QString PointStyleLabel(MapPointStyle style);

/**
 * @brief Human-readable label for a time-range mode.
 *
 * @param range Time-range enum value.
 * @return Localized display string.
 */
QString TimeRangeLabel(MapTimeRange range);

/**
 * @brief Human-readable label for a scale-bar unit.
 *
 * @param unit Distance unit.
 * @return Localized display string.
 */
QString DistanceUnitLabel(MapDistanceUnit unit);

/**
 * @brief Short attribution line for a basemap preset.
 *
 * @param layer Basemap enum value.
 * @return Provider credit, empty for a custom URL.
 */
QString BaseLayerAttribution(MapBaseLayer layer);

/**
 * @brief Human-readable label for a map click tool.
 *
 * @param tool Edit tool.
 * @return Localized display string.
 */
QString EditToolLabel(MapEditTool tool);

}  // namespace map
}  // namespace autoviz
