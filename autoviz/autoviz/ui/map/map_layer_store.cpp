/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/map/map_layer_store.hpp"

#include <algorithm>

#include <QtMath>

namespace autoviz {
namespace map {
namespace {

constexpr int kMaxTrailPoints = 10000;
constexpr double kEarthRadiusMeters = 6378137.0;

constexpr double kPi = 3.14159265358979323846;

double DegToRad(double degrees) { return degrees * kPi / 180.0; }

}  // namespace

void MapLayerStore::clear() {
  QMutexLocker locker(&mutex_);
  channels_.clear();
}

void MapLayerStore::removeChannel(const QString& channel) {
  QMutexLocker locker(&mutex_);
  channels_.remove(channel);
}

void MapLayerStore::updateLayerStyle(const QString& channel,
                                     const MapTopicLayerConfig& style) {
  QMutexLocker locker(&mutex_);
  auto it = channels_.find(channel);
  if (it != channels_.end()) {
    it->style = style;
  }
}

void MapLayerStore::deriveVelocity(ChannelData* data, MapGeoPoint* point) const {
  if (data == nullptr || point == nullptr || !data->has_last_point) {
    return;
  }
  const MapGeoPoint& previous = data->last_point;
  if (point->timestamp_ns <= previous.timestamp_ns) {
    return;
  }
  const double dt =
      static_cast<double>(point->timestamp_ns - previous.timestamp_ns) * 1e-9;
  if (dt <= 0.0) {
    return;
  }
  const double lat_mid =
      DegToRad((previous.latitude + point->latitude) * 0.5);
  const double dlat = DegToRad(point->latitude - previous.latitude);
  const double dlon = DegToRad(point->longitude - previous.longitude);
  const double north = dlat * kEarthRadiusMeters;
  const double east = dlon * kEarthRadiusMeters * std::cos(lat_mid);
  point->velocity_north_mps = north / dt;
  point->velocity_east_mps = east / dt;
}

void MapLayerStore::ingest(const QString& channel, const MapTopicLayerConfig& style,
                           const MapIngestResult& result, quint64 now_ns) {
  QMutexLocker locker(&mutex_);
  ChannelData& data = channels_[channel];
  data.style = style;

  if (!result.lines.isEmpty()) {
    data.lines = result.lines;
  }
  if (!result.polygons.isEmpty()) {
    data.polygons = result.polygons;
  }

  if (result.append_trail) {
    for (MapGeoPoint point : result.points) {
      if (point.timestamp_ns == 0) {
        point.timestamp_ns = now_ns;
      }
      deriveVelocity(&data, &point);
      data.trail.push_back(point);
      data.previous_point = data.last_point;
      data.has_previous_point = data.has_last_point;
      data.last_point = point;
      data.has_last_point = true;
    }
    while (data.trail.size() > kMaxTrailPoints) {
      data.trail.pop_front();
    }
  } else if (!result.points.isEmpty()) {
    data.snapshot_points = result.points;
    data.last_point = result.points.back();
    data.has_last_point = true;
  }
}

QVector<MapGeoPoint> MapLayerStore::filteredPoints(const ChannelData& data,
                                                   quint64 now_ns) const {
  switch (data.style.time_range) {
    case MapTimeRange::kLatest:
      if (data.has_last_point) {
        return {data.last_point};
      }
      return {};
    case MapTimeRange::kLastNSeconds: {
      const quint64 cutoff =
          now_ns - static_cast<quint64>(data.style.time_range_seconds * 1e9);
      QVector<MapGeoPoint> filtered;
      filtered.reserve(static_cast<int>(data.trail.size()) +
                       data.snapshot_points.size());
      for (const MapGeoPoint& point : data.trail) {
        if (point.timestamp_ns >= cutoff) {
          filtered.push_back(point);
        }
      }
      for (const MapGeoPoint& point : data.snapshot_points) {
        filtered.push_back(point);
      }
      return filtered;
    }
    case MapTimeRange::kAll:
      break;
  }

  QVector<MapGeoPoint> all;
  all.reserve(static_cast<int>(data.trail.size()) + data.snapshot_points.size());
  for (const MapGeoPoint& point : data.trail) {
    all.push_back(point);
  }
  for (const MapGeoPoint& point : data.snapshot_points) {
    all.push_back(point);
  }
  return all;
}

QVector<MapRenderLayerSnapshot> MapLayerStore::snapshot(quint64 now_ns) const {
  QMutexLocker locker(&mutex_);
  QVector<MapRenderLayerSnapshot> layers;
  layers.reserve(channels_.size());
  for (auto it = channels_.cbegin(); it != channels_.cend(); ++it) {
    if (!it.value().style.enabled || it.key().isEmpty()) {
      continue;
    }
    MapRenderLayerSnapshot layer;
    layer.channel = it.key();
    layer.style = it.value().style;
    layer.points = filteredPoints(it.value(), now_ns);
    layer.lines = it.value().lines;
    layer.polygons = it.value().polygons;
    layers.push_back(layer);
  }
  return layers;
}

MapFollowTarget MapLayerStore::followTarget(const QString& follow_channel,
                                            quint64 now_ns) const {
  MapFollowTarget target;
  if (follow_channel.isEmpty()) {
    return target;
  }
  QMutexLocker locker(&mutex_);
  const auto found = channels_.constFind(follow_channel);
  if (found == channels_.cend() || !found->has_last_point) {
    return target;
  }
  const QVector<MapGeoPoint> points = filteredPoints(*found, now_ns);
  if (points.isEmpty()) {
    return target;
  }
  const MapGeoPoint& latest = points.back();
  target.latitude = latest.latitude;
  target.longitude = latest.longitude;
  target.valid = true;
  return target;
}

}  // namespace map
}  // namespace autoviz
