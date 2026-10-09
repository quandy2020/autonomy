/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/map/map_types.hpp"

namespace autoviz {
namespace map {

MapPanelConfig DefaultMapPanelConfig() {
  MapPanelConfig config;
  config.title = QString();
  // Prefer CARTO over openstreetmap.org — OSM tile TLS is often reset/blocked
  // in CN and some corporate networks, leaving the Map panel blank.
  config.base_layer = MapBaseLayer::kCartoVoyager;
  return config;
}

QString BaseLayerTileUrlTemplate(MapBaseLayer layer, const QString& custom_url) {
  switch (layer) {
    case MapBaseLayer::kStreet:
      // OSM-sourced street tiles via CARTO (openstreetmap.org TLS often fails).
      return QStringLiteral(
          "https://basemaps.cartocdn.com/rastertiles/voyager/{z}/{x}/{y}.png");
    case MapBaseLayer::kSatellite:
      return QStringLiteral(
          "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/"
          "MapServer/tile/{z}/{y}/{x}");
    case MapBaseLayer::kShadedRelief:
      // OpenTopoMap is frequently unreachable; Esri terrain is a stable relief
      // basemap with the same UX role.
      return QStringLiteral(
          "https://server.arcgisonline.com/ArcGIS/rest/services/"
          "World_Terrain_Base/MapServer/tile/{z}/{y}/{x}");
    case MapBaseLayer::kCustom:
      return custom_url.trimmed();
    case MapBaseLayer::kEsriStreet:
      return QStringLiteral(
          "https://server.arcgisonline.com/ArcGIS/rest/services/World_Street_Map/"
          "MapServer/tile/{z}/{y}/{x}");
    case MapBaseLayer::kEsriTerrain:
      return QStringLiteral(
          "https://server.arcgisonline.com/ArcGIS/rest/services/World_Terrain_Base/"
          "MapServer/tile/{z}/{y}/{x}");
    case MapBaseLayer::kCartoVoyager:
      return QStringLiteral(
          "https://basemaps.cartocdn.com/rastertiles/voyager/{z}/{x}/{y}.png");
    case MapBaseLayer::kJapanStandard:
      return QStringLiteral(
          "https://cyberjapandata.gsi.go.jp/xyz/std/{z}/{x}/{y}.png");
    case MapBaseLayer::kAmapStreet:
      // {s} → 1..4. Tiles are GCJ-02; WGS-84 overlays may show China offset.
      return QStringLiteral(
          "https://webrd0{s}.is.autonavi.com/appmaptile?lang=zh_cn&size=1&"
          "scale=1&style=8&x={x}&y={y}&z={z}");
    case MapBaseLayer::kAmapSatellite:
      return QStringLiteral(
          "https://webst0{s}.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&"
          "z={z}");
    case MapBaseLayer::kBingRoad:
      return QStringLiteral(
          "https://t{s}.ssl.ak.dynamic.tiles.virtualearth.net/comp/ch/"
          "{quadkey}?mkt=zh-CN&it=G,L&n=z");
    case MapBaseLayer::kBingAerial:
      return QStringLiteral(
          "https://t{s}.ssl.ak.dynamic.tiles.virtualearth.net/comp/ch/"
          "{quadkey}?mkt=en-US&it=A&n=z");
  }
  return {};
}

int BaseLayerMaxNativeZoom(MapBaseLayer layer) {
  switch (layer) {
    case MapBaseLayer::kStreet:
      return 19;
    case MapBaseLayer::kSatellite:
      return 22;
    case MapBaseLayer::kShadedRelief:
      return 17;
    case MapBaseLayer::kCustom:
      return 22;
    case MapBaseLayer::kEsriStreet:
      return 19;
    case MapBaseLayer::kEsriTerrain:
      return 13;
    case MapBaseLayer::kCartoVoyager:
      return 20;
    case MapBaseLayer::kJapanStandard:
      return 18;
    case MapBaseLayer::kAmapStreet:
      return 18;
    case MapBaseLayer::kAmapSatellite:
      return 18;
    case MapBaseLayer::kBingRoad:
      return 19;
    case MapBaseLayer::kBingAerial:
      return 19;
  }
  return 19;
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
    case MapBaseLayer::kEsriStreet:
      return QStringLiteral("Esri Street");
    case MapBaseLayer::kEsriTerrain:
      return QStringLiteral("Esri Terrain");
    case MapBaseLayer::kCartoVoyager:
      return QStringLiteral("CARTO Voyager");
    case MapBaseLayer::kJapanStandard:
      return QStringLiteral("Japan GSI");
    case MapBaseLayer::kAmapStreet:
      return QStringLiteral("Amap Street");
    case MapBaseLayer::kAmapSatellite:
      return QStringLiteral("Amap Satellite");
    case MapBaseLayer::kBingRoad:
      return QStringLiteral("Bing Road");
    case MapBaseLayer::kBingAerial:
      return QStringLiteral("Bing Aerial");
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

QString DistanceUnitLabel(MapDistanceUnit unit) {
  switch (unit) {
    case MapDistanceUnit::kMeters:
      return QStringLiteral("Meters");
    case MapDistanceUnit::kFeet:
      return QStringLiteral("Feet");
  }
  return {};
}

QString BaseLayerAttribution(MapBaseLayer layer) {
  switch (layer) {
    case MapBaseLayer::kStreet:
      return QStringLiteral("© OpenStreetMap");
    case MapBaseLayer::kSatellite:
      return QStringLiteral("© Esri");
    case MapBaseLayer::kShadedRelief:
      return QStringLiteral("© Esri");
    case MapBaseLayer::kCustom:
      return {};
    case MapBaseLayer::kEsriStreet:
    case MapBaseLayer::kEsriTerrain:
      return QStringLiteral("© Esri");
    case MapBaseLayer::kCartoVoyager:
      return QStringLiteral("© CARTO © OpenStreetMap");
    case MapBaseLayer::kJapanStandard:
      return QStringLiteral("© GSI Japan");
    case MapBaseLayer::kAmapStreet:
    case MapBaseLayer::kAmapSatellite:
      return QStringLiteral("© Amap");
    case MapBaseLayer::kBingRoad:
    case MapBaseLayer::kBingAerial:
      return QStringLiteral("© Microsoft Bing");
  }
  return {};
}

QString EditToolLabel(MapEditTool tool) {
  switch (tool) {
    case MapEditTool::kPan:
      return QStringLiteral("Pan");
    case MapEditTool::kWaypoint:
      return QStringLiteral("Waypoint");
    case MapEditTool::kGeofence:
      return QStringLiteral("Geofence");
    case MapEditTool::kRally:
      return QStringLiteral("Rally");
  }
  return {};
}

}  // namespace map
}  // namespace autoviz
