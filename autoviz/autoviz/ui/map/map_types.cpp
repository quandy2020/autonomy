/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/map/map_types.hpp"

namespace autoviz {
namespace map {

MapPanelConfig DefaultMapPanelConfig() {
  MapPanelConfig config;
  config.title = QString();
  return config;
}

QString BaseLayerTileUrlTemplate(MapBaseLayer layer, const QString& custom_url) {
  switch (layer) {
    case MapBaseLayer::kStreet:
      return QStringLiteral("https://tile.openstreetmap.org/{z}/{x}/{y}.png");
    case MapBaseLayer::kSatellite:
      return QStringLiteral(
          "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/"
          "MapServer/tile/{z}/{y}/{x}");
    case MapBaseLayer::kShadedRelief:
      return QStringLiteral("https://tile.opentopomap.org/{z}/{x}/{y}.png");
    case MapBaseLayer::kCustom:
      return custom_url.trimmed();
  }
  return {};
}

QString BaseLayerLabel(MapBaseLayer layer) {
  switch (layer) {
    case MapBaseLayer::kStreet:
      return QStringLiteral("Street");
    case MapBaseLayer::kSatellite:
      return QStringLiteral("Satellite");
    case MapBaseLayer::kShadedRelief:
      return QStringLiteral("Shaded relief");
    case MapBaseLayer::kCustom:
      return QStringLiteral("Custom URL");
  }
  return {};
}

QString PointStyleLabel(MapPointStyle style) {
  switch (style) {
    case MapPointStyle::kDot:
      return QStringLiteral("Dot");
    case MapPointStyle::kArrow:
      return QStringLiteral("Arrowhead");
    case MapPointStyle::kDiamond:
      return QStringLiteral("Diamond");
    case MapPointStyle::kSquare:
      return QStringLiteral("Square");
    case MapPointStyle::kCross:
      return QStringLiteral("Cross");
  }
  return {};
}

QString TimeRangeLabel(MapTimeRange range) {
  switch (range) {
    case MapTimeRange::kLatest:
      return QStringLiteral("Latest only");
    case MapTimeRange::kLastNSeconds:
      return QStringLiteral("Last N seconds");
    case MapTimeRange::kAll:
      return QStringLiteral("All history");
  }
  return {};
}

}  // namespace map
}  // namespace autoviz
