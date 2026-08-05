/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QHash>
#include <QMutex>
#include <QVector>

#include <deque>
#include <optional>

#include "autoviz/ui/map/map_geojson_parser.hpp"
#include "autoviz/ui/map/map_types.hpp"

namespace autoviz {
namespace map {

struct MapRenderLayerSnapshot {
  QString channel;
  MapTopicLayerConfig style;
  QVector<MapGeoPoint> points;
  QVector<MapGeoLine> lines;
  QVector<MapGeoPolygon> polygons;
};

struct MapFollowTarget {
  double latitude = 0.0;
  double longitude = 0.0;
  bool valid = false;
};

/** Thread-safe store for per-channel geographic features with time filtering. */
class MapLayerStore {
 public:
  void clear();
  void removeChannel(const QString& channel);
  void updateLayerStyle(const QString& channel, const MapTopicLayerConfig& style);
  void ingest(const QString& channel, const MapTopicLayerConfig& style,
              const MapIngestResult& result, quint64 now_ns);

  QVector<MapRenderLayerSnapshot> snapshot(quint64 now_ns) const;
  MapFollowTarget followTarget(const QString& follow_channel, quint64 now_ns) const;

 private:
  struct ChannelData {
    MapTopicLayerConfig style;
    std::deque<MapGeoPoint> trail;
    QVector<MapGeoPoint> snapshot_points;
    QVector<MapGeoLine> lines;
    QVector<MapGeoPolygon> polygons;
    MapGeoPoint last_point;
    bool has_last_point = false;
    MapGeoPoint previous_point;
    bool has_previous_point = false;
  };

  QVector<MapGeoPoint> filteredPoints(const ChannelData& data,
                                      quint64 now_ns) const;
  void deriveVelocity(ChannelData* data, MapGeoPoint* point) const;

  mutable QMutex mutex_;
  QHash<QString, ChannelData> channels_;
};

}  // namespace map
}  // namespace autoviz
