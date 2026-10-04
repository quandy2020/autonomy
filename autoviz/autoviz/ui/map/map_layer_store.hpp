/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file map_layer_store.hpp
 * @brief Thread-safe per-channel store of geographic features with time filters.
 *
 * Ingest path (possibly background) writes via @ref MapLayerStore::ingest();
 * the UI thread reads @ref snapshot() / @ref followTarget() for rendering and
 * camera follow.
 *
 * @see MapPanel
 * @see MapViewportWidget
 * @see MapIngestResult
 */

#pragma once

#include <QHash>
#include <QtMath>
#include <QMutex>
#include <QVector>

#include <deque>
#include <optional>

#include "autoviz/ui/map/map_geojson_parser.hpp"
#include "autoviz/ui/map/map_types.hpp"

namespace autoviz {
namespace map {

/**
 * @struct MapRenderLayerSnapshot
 * @brief Immutable render-ready copy of one channel's features and style.
 */
struct MapRenderLayerSnapshot {
  QString channel;              /**< Source channel name. */
  MapTopicLayerConfig style;    /**< Draw style for this layer. */
  QVector<MapGeoPoint> points;  /**< Time-filtered points for this frame. */
  QVector<MapGeoLine> lines;    /**< Line features. */
  QVector<MapGeoPolygon> polygons;  /**< Polygon features. */
};

/**
 * @struct MapFollowTarget
 * @brief Latest follow-camera lat/lon for a configured follow channel.
 */
struct MapFollowTarget {
  double latitude = 0.0;   /**< Target latitude (degrees). */
  double longitude = 0.0;  /**< Target longitude (degrees). */
  double heading_deg = qQNaN(); /**< Heading in degrees; NaN if unknown. */
  bool valid = false;      /**< @c true when a recent point exists. */
};

/**
 * @class MapLayerStore
 * @brief Thread-safe store for per-channel geographic features with time filtering.
 *
 * Maintains trails, snapshot points, lines, and polygons keyed by channel.
 * @ref snapshot() applies @ref MapTimeRange from each layer's style.
 *
 * @note All public methods take @c mutex_; keep critical sections short.
 */
class MapLayerStore {
 public:
  /**
   * @brief Removes all channels and features.
   */
  void clear();

  /**
   * @brief Drops a single channel's data.
   *
   * @param channel Channel key to remove.
   */
  void removeChannel(const QString& channel);

  /**
   * @brief Updates draw style for an existing (or placeholder) channel.
   *
   * @param channel Channel key.
   * @param style New topic-layer style from panel config.
   */
  void updateLayerStyle(const QString& channel, const MapTopicLayerConfig& style);

  /**
   * @brief Merges an ingest batch into the channel under @p style.
   *
   * @param channel Channel key.
   * @param style Style applied / stored with the channel.
   * @param result Features from @ref MapMessageIngest / GeoJSON parse.
   * @param now_ns Current time for trail / velocity bookkeeping (nanoseconds).
   */
  void ingest(const QString& channel, const MapTopicLayerConfig& style,
              const MapIngestResult& result, quint64 now_ns);

  /**
   * @brief Builds time-filtered snapshots for all channels.
   *
   * @param now_ns Current time used by @ref MapTimeRange filtering.
   * @return One snapshot per known channel (enabled style still returned;
   *         viewport may skip disabled layers).
   */
  QVector<MapRenderLayerSnapshot> snapshot(quint64 now_ns) const;

  /**
   * @brief Resolves the follow-camera target from a channel's latest point.
   *
   * @param follow_channel Channel to follow; empty → invalid target.
   * @param now_ns Current time for freshness checks.
   * @return Follow target with @c valid set appropriately.
   */
  MapFollowTarget followTarget(const QString& follow_channel, quint64 now_ns) const;

 private:
  /**
   * @struct ChannelData
   * @brief Mutable per-channel storage under the store mutex.
   */
  struct ChannelData {
    MapTopicLayerConfig style;           /**< Draw / filter style. */
    std::deque<MapGeoPoint> trail;       /**< Rolling trail for time-range modes. */
    QVector<MapGeoPoint> snapshot_points; /**< Non-trail point snapshot. */
    QVector<MapGeoLine> lines;           /**< Line features. */
    QVector<MapGeoPolygon> polygons;     /**< Polygon features. */
    MapGeoPoint last_point;             /**< Most recent point. */
    bool has_last_point = false;        /**< Whether @c last_point is set. */
    MapGeoPoint previous_point;         /**< Prior point for velocity derivation. */
    bool has_previous_point = false;    /**< Whether @c previous_point is set. */
  };

  /**
   * @brief Applies the channel's @ref MapTimeRange to produce drawable points.
   *
   * @param data Channel storage.
   * @param now_ns Current time (nanoseconds).
   * @return Filtered point list.
   */
  QVector<MapGeoPoint> filteredPoints(const ChannelData& data,
                                      quint64 now_ns) const;

  /**
   * @brief Fills velocity fields on @p point from previous→last motion when needed.
   *
   * @param data Channel storage (previous / last points).
   * @param point Point to annotate in-place.
   */
  void deriveVelocity(ChannelData* data, MapGeoPoint* point) const;

  /** Guards @c channels_. */
  mutable QMutex mutex_;

  /** Per-channel feature storage. */
  QHash<QString, ChannelData> channels_;
};

}  // namespace map
}  // namespace autoviz
