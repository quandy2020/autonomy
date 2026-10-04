/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file geo_map.hpp
 * @brief GeoMap engine: GeoJSON document, hit-testing, and measurement.
 *
 * The slippy canvas stays in @ref MapViewportWidget. GeoMap owns file layers,
 * the selected feature, and the two-point measure tool. Hidden layers are
 * excluded from hit-testing.
 *
 * @see MapViewportWidget
 * @see MapGeoJsonParser
 */

#pragma once

#include <QPointF>
#include <QString>
#include <QVector>

#include <functional>

#include "autoviz/ui/map/map_geojson_parser.hpp"
#include "autoviz/ui/map/map_layer_store.hpp"

namespace autoviz {
namespace map {

/**
 * @enum GeoMapFeatureKind
 * @brief Which geometry a hit or selection refers to.
 */
enum class GeoMapFeatureKind {
  kNone = 0,
  kPoint,
  kLine,
  kPolygon,
};

/**
 * @enum GeoMapLayerStatus
 * @brief Parse outcome for one GeoJSON document layer.
 */
enum class GeoMapLayerStatus {
  kOk = 0,
  kError,
};

/**
 * @struct GeoMapHit
 * @brief One picked feature in screen space.
 */
struct GeoMapHit {
  bool valid = false;
  QString layer_id;
  GeoMapFeatureKind kind = GeoMapFeatureKind::kNone;
  int index = -1;
  double latitude = 0.0;
  double longitude = 0.0;
  QString label;
  QString layer_name;
  QVector<MapAttribute> attributes;
};

/**
 * @struct GeoMapLayer
 * @brief One loaded GeoJSON document.
 */
struct GeoMapLayer {
  QString id;
  QString path;   /**< Absolute path; empty for text that was not a file. */
  QString source; /**< Short label, usually the file name. */
  bool visible = true;
  GeoMapLayerStatus status = GeoMapLayerStatus::kOk;
  QString status_text;
  MapIngestResult features;
};

/**
 * @class GeoMap
 * @brief Document and query engine for the Map panel.
 */
class GeoMap {
 public:
  /**
   * @brief Loads a GeoJSON file as a new layer.
   *
   * A parse failure still inserts a layer with @ref GeoMapLayerStatus::kError
   * so other layers keep drawing.
   *
   * @param path Filesystem path.
   * @param error Optional failure text.
   * @return Layer id, empty when the file cannot be read.
   */
  QString loadFile(const QString& path, QString* error);

  /**
   * @brief Loads GeoJSON text as a new layer.
   *
   * @param text GeoJSON document.
   * @param source Label stored on the layer (path or "drop").
   * @param error Optional parse error.
   * @return New layer id.
   */
  QString loadText(const QString& text, const QString& source, QString* error);

  /**
   * @brief Shows or hides a document layer.
   *
   * @param layer_id Layer id from @ref loadText.
   * @param visible New visibility.
   * @return @c false when the id is unknown.
   */
  bool setLayerVisible(const QString& layer_id, bool visible);

  /**
   * @brief Drops a document layer.
   *
   * @param layer_id Layer id.
   * @return @c false when the id is unknown.
   */
  bool removeLayer(const QString& layer_id);

  /**
   * @brief Reloads the document so it matches @p sources.
   *
   * Existing parses are kept when the path is unchanged. New paths are read
   * from disk. Paths that disappeared are dropped.
   *
   * @param sources Desired files and visibility.
   */
  void syncSources(const QVector<MapGeoJsonSource>& sources);

  /** @return File sources currently held, in layer order. */
  QVector<MapGeoJsonSource> sources() const;

  /** @return Document layers in insertion order. */
  const QVector<GeoMapLayer>& layers() const { return layers_; }

  /**
   * @brief Visible, successfully parsed layers as render snapshots.
   *
   * @c MapRenderLayerSnapshot::channel is the layer id.
   */
  QVector<MapRenderLayerSnapshot> renderLayers() const;

  /**
   * @brief Picks the nearest visible feature.
   *
   * @param screen Cursor in viewport pixels.
   * @param live_layers Channel snapshots already on screen.
   * @param to_screen Projects latitude/longitude to viewport pixels.
   * @return Hit, or invalid when nothing is within the pick tolerance.
   */
  GeoMapHit hitTest(
      const QPointF& screen, const QVector<MapRenderLayerSnapshot>& live_layers,
      const std::function<QPointF(double latitude, double longitude)>& to_screen)
      const;

  void setSelection(const GeoMapHit& hit) { selection_ = hit; }
  const GeoMapHit& selection() const { return selection_; }
  void clearSelection() { selection_ = {}; }

  /** Starts or stops the two-point measure tool. */
  void setMeasuring(bool enabled);
  bool measuring() const { return measuring_; }

  /**
   * @brief Adds a measure vertex. A third point starts a new segment.
   *
   * @param latitude WGS84 latitude.
   * @param longitude WGS84 longitude.
   */
  void addMeasurePoint(double latitude, double longitude);
  void clearMeasure();

  /** @return Measure vertices as (latitude, longitude). */
  const QVector<QPointF>& measurePoints() const { return measure_points_; }

  /**
   * @brief Ground length of the current measure segment.
   *
   * @return Meters, or 0 when fewer than two points exist.
   */
  double measureMeters() const;

  /**
   * @brief Measure length in the panel's distance unit.
   *
   * @param unit Meters or feet.
   * @return Empty when fewer than two points exist.
   */
  QString formatMeasure(MapDistanceUnit unit) const;

  /**
   * @brief One-line status: selection, measure, and document errors.
   *
   * @param unit Unit used for the measure length.
   */
  QString statusLine(MapDistanceUnit unit) const;

 private:
  QVector<GeoMapLayer> layers_;
  GeoMapHit selection_;
  bool measuring_ = false;
  QVector<QPointF> measure_points_;
  int next_id_ = 1;
};

}  // namespace map
}  // namespace autoviz
