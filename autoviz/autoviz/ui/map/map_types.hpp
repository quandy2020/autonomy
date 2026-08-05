/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QColor>
#include <QString>
#include <QVector>

namespace autoviz {
namespace map {

enum class MapBaseLayer {
  kStreet = 0,
  kSatellite = 1,
  kShadedRelief = 2,
  kCustom = 3,
};

enum class MapPointStyle {
  kDot = 0,
  kArrow = 1,
  kDiamond = 2,
  kSquare = 3,
  kCross = 4,
};

enum class MapTimeRange {
  kLatest = 0,
  kLastNSeconds = 1,
  kAll = 2,
};

struct MapOverlayLayerConfig {
  QString name;
  QString tile_url_template;
  double opacity = 0.5;
  bool enabled = true;
};

struct MapTopicLayerConfig {
  QString channel;
  MapPointStyle point_style = MapPointStyle::kDot;
  bool show_heading = true;
  bool show_velocity = false;
  double point_size = 8.0;
  MapTimeRange time_range = MapTimeRange::kLatest;
  double time_range_seconds = 30.0;
  double layer_opacity = 1.0;
  QColor color;
  bool enabled = true;
};

struct MapPanelConfig {
  QString title;
  MapBaseLayer base_layer = MapBaseLayer::kStreet;
  QString custom_tile_url;
  QString follow_channel;
  double center_latitude = 39.9042;
  double center_longitude = 116.4074;
  double zoom = 14.0;
  QVector<MapOverlayLayerConfig> overlay_layers;
  QVector<MapTopicLayerConfig> topic_layers;
};

MapPanelConfig DefaultMapPanelConfig();
QString BaseLayerTileUrlTemplate(MapBaseLayer layer, const QString& custom_url);
QString BaseLayerLabel(MapBaseLayer layer);
QString PointStyleLabel(MapPointStyle style);
QString TimeRangeLabel(MapTimeRange range);

}  // namespace map
}  // namespace autoviz
