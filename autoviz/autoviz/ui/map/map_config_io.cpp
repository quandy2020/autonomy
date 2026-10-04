/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/map/map_config_io.hpp"

#include <QColor>

namespace autoviz {
namespace map {
namespace {

int ClampEnum(int value, int max_value) {
  if (value < 0) {
    return 0;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

common::MapOverlayPersistConfig ToOverlay(const MapOverlayLayerConfig& layer) {
  common::MapOverlayPersistConfig out;
  out.name = layer.name.toStdString();
  out.tile_url_template = layer.tile_url_template.toStdString();
  out.opacity = layer.opacity;
  out.enabled = layer.enabled;
  return out;
}

MapOverlayLayerConfig FromOverlay(const common::MapOverlayPersistConfig& layer) {
  MapOverlayLayerConfig out;
  out.name = QString::fromStdString(layer.name);
  out.tile_url_template = QString::fromStdString(layer.tile_url_template);
  out.opacity = layer.opacity;
  out.enabled = layer.enabled;
  return out;
}

common::MapTopicLayerPersistConfig ToTopic(const MapTopicLayerConfig& layer) {
  common::MapTopicLayerPersistConfig out;
  out.channel = layer.channel.toStdString();
  out.point_style = static_cast<int>(layer.point_style);
  out.show_heading = layer.show_heading;
  out.show_velocity = layer.show_velocity;
  out.point_size = layer.point_size;
  out.time_range = static_cast<int>(layer.time_range);
  out.time_range_seconds = layer.time_range_seconds;
  out.layer_opacity = layer.layer_opacity;
  if (layer.color.isValid()) {
    out.color = layer.color.name().toStdString();
  }
  out.enabled = layer.enabled;
  return out;
}

MapTopicLayerConfig FromTopic(const common::MapTopicLayerPersistConfig& layer) {
  MapTopicLayerConfig out;
  out.channel = QString::fromStdString(layer.channel);
  out.point_style = static_cast<MapPointStyle>(
      ClampEnum(layer.point_style, static_cast<int>(MapPointStyle::kCross)));
  out.show_heading = layer.show_heading;
  out.show_velocity = layer.show_velocity;
  out.point_size = layer.point_size > 0.0 ? layer.point_size : 8.0;
  out.time_range = static_cast<MapTimeRange>(
      ClampEnum(layer.time_range, static_cast<int>(MapTimeRange::kAll)));
  out.time_range_seconds =
      layer.time_range_seconds > 0.0 ? layer.time_range_seconds : 30.0;
  out.layer_opacity = layer.layer_opacity;
  if (!layer.color.empty()) {
    out.color = QColor(QString::fromStdString(layer.color));
  }
  out.enabled = layer.enabled;
  return out;
}

}  // namespace

common::MapPanelPersistConfig ToPersistConfig(const QString& object_name,
                                              const MapPanelConfig& config,
                                              bool settings_visible) {
  common::MapPanelPersistConfig persist;
  persist.object_name = object_name.toStdString();
  persist.title = config.title.toStdString();
  persist.base_layer = static_cast<int>(config.base_layer);
  persist.custom_tile_url = config.custom_tile_url.toStdString();
  persist.follow_channel = config.follow_channel.toStdString();
  persist.gcs_channel = config.gcs_channel.toStdString();
  persist.distance_unit = static_cast<int>(config.distance_unit);
  persist.center_latitude = config.center_latitude;
  persist.center_longitude = config.center_longitude;
  persist.zoom = config.zoom;
  persist.overlay_layers.reserve(
      static_cast<std::size_t>(config.overlay_layers.size()));
  for (const MapOverlayLayerConfig& layer : config.overlay_layers) {
    persist.overlay_layers.push_back(ToOverlay(layer));
  }
  persist.topic_layers.reserve(
      static_cast<std::size_t>(config.topic_layers.size()));
  for (const MapTopicLayerConfig& layer : config.topic_layers) {
    persist.topic_layers.push_back(ToTopic(layer));
  }
  persist.geojson_sources.reserve(
      static_cast<std::size_t>(config.geojson_sources.size()));
  for (const MapGeoJsonSource& source : config.geojson_sources) {
    common::MapGeoJsonPersistConfig out;
    out.path = source.path.toStdString();
    out.visible = source.visible;
    persist.geojson_sources.push_back(std::move(out));
  }
  auto copy_points = [](const QVector<MapPlanPoint>& points) {
    std::vector<common::MapPlanPointPersistConfig> out;
    out.reserve(static_cast<std::size_t>(points.size()));
    for (const MapPlanPoint& point : points) {
      common::MapPlanPointPersistConfig stored;
      stored.latitude = point.latitude;
      stored.longitude = point.longitude;
      out.push_back(stored);
    }
    return out;
  };
  persist.waypoints = copy_points(config.waypoints);
  persist.geofence = copy_points(config.geofence);
  persist.rally_points = copy_points(config.rally_points);
  persist.edit_tool = static_cast<int>(config.edit_tool);
  persist.survey_spacing_m = config.survey_spacing_m;
  persist.corridor_width_m = config.corridor_width_m;
  persist.structure_radius_m = config.structure_radius_m;
  persist.settings_visible = settings_visible;
  return persist;
}

MapPanelConfig FromPersistConfig(const common::MapPanelPersistConfig& persist) {
  MapPanelConfig config = DefaultMapPanelConfig();
  config.title = QString::fromStdString(persist.title);
  config.base_layer = static_cast<MapBaseLayer>(
      ClampEnum(persist.base_layer, static_cast<int>(MapBaseLayer::kJapanStandard)));
  config.custom_tile_url = QString::fromStdString(persist.custom_tile_url);
  config.follow_channel = QString::fromStdString(persist.follow_channel);
  config.gcs_channel = QString::fromStdString(persist.gcs_channel);
  config.distance_unit = static_cast<MapDistanceUnit>(
      ClampEnum(persist.distance_unit, static_cast<int>(MapDistanceUnit::kFeet)));
  config.center_latitude = persist.center_latitude;
  config.center_longitude = persist.center_longitude;
  if (persist.zoom > 0.0) {
    config.zoom = persist.zoom;
  }
  config.overlay_layers.reserve(
      static_cast<int>(persist.overlay_layers.size()));
  for (const common::MapOverlayPersistConfig& layer : persist.overlay_layers) {
    config.overlay_layers.push_back(FromOverlay(layer));
  }
  config.topic_layers.reserve(static_cast<int>(persist.topic_layers.size()));
  for (const common::MapTopicLayerPersistConfig& layer : persist.topic_layers) {
    config.topic_layers.push_back(FromTopic(layer));
  }
  config.geojson_sources.reserve(static_cast<int>(persist.geojson_sources.size()));
  for (const common::MapGeoJsonPersistConfig& source : persist.geojson_sources) {
    MapGeoJsonSource out;
    out.path = QString::fromStdString(source.path);
    out.visible = source.visible;
    config.geojson_sources.push_back(out);
  }
  config.edit_tool = static_cast<MapEditTool>(
      ClampEnum(persist.edit_tool, static_cast<int>(MapEditTool::kRally)));
  config.waypoints.reserve(static_cast<int>(persist.waypoints.size()));
  for (const common::MapPlanPointPersistConfig& point : persist.waypoints) {
    config.waypoints.push_back(MapPlanPoint{point.latitude, point.longitude});
  }
  config.geofence.reserve(static_cast<int>(persist.geofence.size()));
  for (const common::MapPlanPointPersistConfig& point : persist.geofence) {
    config.geofence.push_back(MapPlanPoint{point.latitude, point.longitude});
  }
  config.rally_points.reserve(static_cast<int>(persist.rally_points.size()));
  for (const common::MapPlanPointPersistConfig& point : persist.rally_points) {
    config.rally_points.push_back(MapPlanPoint{point.latitude, point.longitude});
  }
  if (persist.survey_spacing_m > 0.0) {
    config.survey_spacing_m = persist.survey_spacing_m;
  }
  if (persist.corridor_width_m > 0.0) {
    config.corridor_width_m = persist.corridor_width_m;
  }
  if (persist.structure_radius_m > 0.0) {
    config.structure_radius_m = persist.structure_radius_m;
  }
  return config;
}

}  // namespace map
}  // namespace autoviz
