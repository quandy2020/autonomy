/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file map_viewport_widget.hpp
 * @brief Interactive Web Mercator map canvas: tiles, geo layers, pan/zoom.
 *
 * Owned by @ref MapPanel. Requests tiles from @ref MapTileCache, draws
 * @ref MapRenderLayerSnapshot features, and optionally follows
 * @ref MapFollowTarget until the user pans.
 *
 * @see MapPanel
 * @see MapTileCache
 * @see MapProjection
 */

#pragma once

#include <QSet>
#include <QWidget>

#include "autoviz/ui/map/geo_map.hpp"
#include "autoviz/ui/map/map_layer_store.hpp"
#include "autoviz/ui/map/map_tile_cache.hpp"
#include "autoviz/ui/map/map_types.hpp"

class QTimer;

namespace autoviz {
namespace map {

class MapToolbar;

/**
 * @class MapViewportWidget
 * @brief Pan/zoom slippy-map view with basemap, overlays, and topic layers.
 *
 * ## Interaction
 *
 * - **Drag:** pan the map (disables follow until the center button resumes it)
 * - **Wheel / double-click:** zoom toward the cursor, keeping that geo point fixed
 * - **Scale bar:** ground distance in the lower-left corner
 * - **viewChanged:** notifies the panel to persist center/zoom
 *
 * Emits @ref viewChanged() when the camera moves.
 */
class MapViewportWidget : public QWidget {
  Q_OBJECT

 public:
  /**
   * @brief Constructs the viewport and wires tile-cache ready signals.
   *
   * @param parent Qt parent widget.
   */
  explicit MapViewportWidget(QWidget* parent = nullptr);

  /**
   * @brief Applies panel config (basemap, overlays, center/zoom, follow).
   *
   * @param config Map panel configuration.
   */
  void setConfig(const MapPanelConfig& config);

  /**
   * @brief Returns the config last applied / updated from view changes.
   *
   * @return Copy of @c config_.
   */
  MapPanelConfig config() const { return config_; }

  /** @return @c true while a left-button pan is active. */
  bool isPanning() const { return panning_; }

  /**
   * @brief Replaces the geo layers drawn above the tiles.
   *
   * @param layers Snapshots from @ref MapLayerStore::snapshot().
   */
  void setLayers(const QVector<MapRenderLayerSnapshot>& layers);

  /**
   * @brief Sets the optional follow-camera target.
   *
   * When valid and the user has not manually panned, the view centers on the
   * target.
   *
   * @param target Follow lat/lon from the layer store.
   */
  void setFollowTarget(const MapFollowTarget& target);

  /**
   * @brief Sets the ground-station marker.
   *
   * @param target Latest GCS lat/lon. Invalid hides the marker.
   */
  void setGcsTarget(const MapFollowTarget& target);

  /**
   * @brief Loads a GeoJSON file into the GeoMap document.
   *
   * @param path Filesystem path.
   * @return @c false when the file cannot be read.
   */
  bool loadGeoJsonFile(const QString& path);

  /**
   * @brief Downloads basemap tiles covering the current view into the disk cache.
   *
   * @param min_zoom Lowest tile zoom to fetch.
   * @param max_zoom Highest tile zoom to fetch.
   */
  void downloadVisibleTiles(int min_zoom, int max_zoom);

  /**
   * @brief Recenters on the follow target, or fits visible layers / plan.
   */
  void recenter();

  /** @return GeoJSON files currently held by GeoMap. */
  QVector<MapGeoJsonSource> geoJsonSources() const { return geo_map_.sources(); }

  /**
   * @brief Moves one plan vertex and refreshes the attribute inspector.
   *
   * @param kind 0 waypoint, 1 geofence, 2 rally.
   * @param index Vertex index.
   * @param latitude New latitude.
   * @param longitude New longitude.
   */
  void setPlanVertex(int kind, int index, double latitude, double longitude);

  /**
   * @brief Deletes one plan vertex.
   *
   * @param kind 0 waypoint, 1 geofence, 2 rally.
   * @param index Vertex index.
   */
  void removePlanVertex(int kind, int index);

 signals:
  /**
   * @brief Emitted when the user (or follow) changes center/zoom.
   *
   * @param latitude Center latitude.
   * @param longitude Center longitude.
   * @param zoom Zoom level.
   */
  void viewChanged(double latitude, double longitude, double zoom);

  /**
   * @brief Selection, measure length, or GeoJSON errors for the status bar.
   *
   * @param text Empty when there is nothing extra to show.
   */
  void mapStatusChanged(const QString& text);

  /**
   * @brief GeoJSON file list changed and should be stored with the panel.
   */
  void geoJsonSourcesChanged();

  /**
   * @brief Asks the panel to hide a live topic layer.
   *
   * Document layers are hidden inside GeoMap and do not use this signal.
   *
   * @param channel Topic channel name.
   */
  void hideTopicLayerRequested(const QString& channel);

  /**
   * @brief Waypoints, geofence, or rally points changed.
   */
  void planChanged();

  /**
   * @brief The attribute inspector should show @p info.
   *
   * @param info Selected feature or plan vertex. Invalid clears the inspector.
   */
  void selectionChanged(const MapSelectionInfo& info);

  /**
   * @brief Cursor geo position for the status bar.
   *
   * @param latitude Cursor latitude, or NaN when the cursor left the view.
   * @param longitude Cursor longitude.
   */
  void cursorGeoChanged(double latitude, double longitude);

 protected:
  /**
   * @brief Paints basemap tiles, overlays, and geo layers.
   *
   * @param event Paint event.
   */
  void paintEvent(QPaintEvent* event) override;

  /**
   * @brief Requests tiles for the new viewport size.
   *
   * @param event Resize event.
   */
  void resizeEvent(QResizeEvent* event) override;

  /**
   * @brief Zooms toward the cursor and emits @ref viewChanged().
   *
   * @param event Wheel event.
   */
  void wheelEvent(QWheelEvent* event) override;

  /**
   * @brief Begins a pan gesture.
   *
   * @param event Mouse press event.
   */
  void mousePressEvent(QMouseEvent* event) override;

  /**
   * @brief Updates pan offset while dragging.
   *
   * @param event Mouse move event.
   */
  void mouseMoveEvent(QMouseEvent* event) override;

  /**
   * @brief Ends pan and emits @ref viewChanged().
   *
   * @param event Mouse release event.
   */
  void mouseReleaseEvent(QMouseEvent* event) override;

  /**
   * @brief Zooms in one level, keeping the point under the cursor fixed.
   *
   * @param event Mouse double-click event.
   */
  void mouseDoubleClickEvent(QMouseEvent* event) override;

  /**
   * @brief Clears the hover readout when the cursor leaves.
   *
   * @param event Leave event.
   */
  void leaveEvent(QEvent* event) override;

  /**
   * @brief Esc / tool shortcuts / undo plan point.
   *
   * @param event Key event.
   */
  void keyPressEvent(QKeyEvent* event) override;

  /**
   * @brief Pinch-zoom, matching QGC's pinch handler.
   *
   * @param event Gesture event.
   * @return @c true when the gesture was accepted.
   */
  bool event(QEvent* event) override;

 private slots:
  /**
   * @brief Repaints when a requested tile arrives in the cache.
   *
   * @param coord Tile that became ready.
   */
  void onTileReady(const MapTileCoord& coord);

  /** @brief Updates offline-download progress when a tile is missing. */
  void onTileUnavailable(const MapTileCoord& coord);

 private:
  /**
   * @brief World-pixel position of the current map center at integer zoom.
   *
   * @return Center in global tile mosaic coordinates.
   */
  QPointF centerWorldPixel() const;

  /** Enqueue HTTP fetches for all visible (plus margin) tiles. */
  void requestVisibleTiles();

  /**
   * @brief Draws tiles from @p cache using @p url_template at @p opacity.
   *
   * @param painter Active painter.
   * @param cache Tile cache (base or overlay).
   * @param url_template Tile URL template.
   * @param opacity Layer opacity in \[0, 1\].
   */
  void drawTiles(QPainter* painter, MapTileCache* cache, const QString& url_template,
                 double opacity);

  /** Draws all enabled topic/geo layers from @c layers_. */
  void drawLayers(QPainter* painter);

  /**
   * @brief Draws a single geo point with style-dependent glyph.
   *
   * @param painter Active painter.
   * @param point Geographic point (heading/velocity optional).
   * @param style Layer style.
   * @param screen_pos Already-projected screen position.
   */
  void drawPoint(QPainter* painter, const MapGeoPoint& point,
                 const MapTopicLayerConfig& style, const QPointF& screen_pos);

  /** Draws a QGC-style distance scale in the lower-left corner. */
  void drawScaleBar(QPainter* painter) const;

  /** Draws a north-up compass rose in the upper-left corner. */
  void drawCompass(QPainter* painter) const;

  /** Draws the basemap provider credit. */
  void drawAttribution(QPainter* painter) const;

  /** Glass callout for the selected feature or plan vertex. */
  void drawAttributeCard(QPainter* painter) const;

  /** Draws the follow-channel vehicle glyph and heading. */
  void drawVehicle(QPainter* painter) const;

  /** Draws the ground-station marker. */
  void drawGcs(QPainter* painter) const;

  /** Draws waypoints, the geofence, and rally points from @c config_. */
  void drawPlan(QPainter* painter) const;

  /** Dashed preview from the last plan/measure vertex to the cursor. */
  void drawHoverPreview(QPainter* painter) const;

  /** Drops one coordinate from the offline-download set and refreshes status. */
  void noteDownloadFinished(const MapTileCoord& coord);

  /** Removes the last vertex of the active plan layer. */
  bool undoLastPlanPoint();

  /**
   * @brief Hits a plan vertex within @p tolerance_px.
   *
   * @param screen Cursor in viewport pixels.
   * @param tolerance_px Pick radius.
   * @param[out] kind 0=waypoint, 1=geofence, 2=rally.
   * @param[out] index Vertex index.
   * @return @c true when a vertex is under the cursor.
   */
  bool hitPlanVertex(const QPointF& screen, double tolerance_px, int* kind,
                     int* index) const;

  /** Two-point measure segment. */
  void drawMeasure(QPainter* painter) const;

  /** Click that was not a pan: select or add a measure vertex. */
  void handleMapClick(const QPoint& screen_pos);

  void publishMapStatus();

  /** Arrowhead on the last segment of a mission line. */
  void drawMissionArrow(QPainter* painter, const QPointF& from,
                        const QPointF& to, const QColor& color) const;

  /** Right-click / press-and-hold menu at a map pixel. */
  void showMapMenu(const QPoint& screen_pos);

  /** Starts the press-and-hold timer. */
  void armPressAndHold(const QPoint& screen_pos);

  /**
   * @brief Tile z closest to one source pixel per device pixel.
   *
   * Rounded toward the camera, then clamped to the basemap's native max zoom
   * so a missing higher level is not replaced by an upscaled quadrant.
   * @ref tileScale() maps that raster back onto the camera.
   *
   * @return Tile z.
   */
  int tileZoom() const;

  /**
   * @brief Logical-pixel scale of one tile at the current camera zoom.
   *
   * @return @c 2^(zoom - tileZoom()).
   */
  double tileScale() const;

  /**
   * @brief Inverse of @ref latLonToScreen.
   *
   * @param screen Widget pixel.
   * @return Latitude in x, longitude in y.
   */
  QPointF screenToLatLon(const QPointF& screen) const;

  /**
   * @brief Moves the camera so @p latitude / @p longitude stays under @p screen.
   *
   * Same idea as QGC @c Map.alignCoordinateToPoint.
   *
   * @param latitude Anchor latitude.
   * @param longitude Anchor longitude.
   * @param screen Screen point that should keep showing the anchor.
   */
  void alignLatLonToScreen(double latitude, double longitude, const QPointF& screen);

  /** Recenters on the follow target, or fits all layer geometry. */
  void centerOnContent();

  /**
   * @brief Frames @p fit_layers in the viewport.
   *
   * @param fit_layers Layers whose vertices define the bounds.
   */
  void fitToLayers(const QVector<MapRenderLayerSnapshot>& fit_layers);

  /** Lays out the floating tool strip on the right edge. */
  void positionToolbar();

  /** Updates the mouse cursor for the active edit / measure tool. */
  void syncCursor();

  /** Keeps the overlay toolbar in sync with @c config_ / GeoMap. */
  void syncToolbar();

  /**
   * @brief Projects WGS84 lat/lon to widget pixel coordinates.
   *
   * @param latitude Latitude (degrees).
   * @param longitude Longitude (degrees).
   * @return Screen position.
   */
  QPointF latLonToScreen(double latitude, double longitude) const;

  /**
   * @brief One on-screen tile, including copies past the antimeridian.
   *
   * @c coord is wrapped into the tile pyramid. @c column may lie outside
   * @c [0, 2^z) so the same tile can be drawn again and cover a wide view.
   */
  struct TilePlacement {
    MapTileCoord coord;
    int column = 0;
    int row = 0;
  };

  /** Tiles covering the viewport, repeated horizontally when needed. */
  QVector<TilePlacement> visibleTilePlacements() const;

  /** Camera / basemap / overlay / topic config. */
  MapPanelConfig config_;

  /** Latest geo layer snapshots to draw. */
  QVector<MapRenderLayerSnapshot> layers_;

  /** GeoJSON document, picking, and measurement. */
  GeoMap geo_map_;

  /** Optional follow-camera target. */
  MapFollowTarget follow_target_;

  /** Optional ground-station marker. */
  MapFollowTarget gcs_target_;

  /** Press-and-hold (QGC 600 ms) before the map menu. */
  QTimer* hold_timer_ = nullptr;

  /** Screen position captured when the hold timer started. */
  QPoint hold_press_pos_;

  /** Geo point under the pinch centroid at gesture start. */
  QPointF pinch_anchor_;

  /** Screen point of the pinch centroid at gesture start. */
  QPointF pinch_screen_;

  /** @c true after a pinch gesture has started. */
  bool pinch_active_ = false;

  /** LRU/HTTP cache for the basemap. */
  MapTileCache base_tile_cache_;

  /** LRU/HTTP cache for overlay tile layers. */
  MapTileCache overlay_tile_cache_;

  /** Whether a pan drag is in progress. */
  bool panning_ = false;

  /** Tiles still outstanding for an offline download. */
  QSet<MapTileCoord> download_pending_;

  /** Tile count for the active offline download, including ones already cached. */
  int download_total_ = 0;

  /**
   * Geographic point under the cursor at left-press.
   * x is latitude, y is longitude, matching @ref screenToLatLon().
   */
  QPointF pan_anchor_;

  /**
   * @brief Set when the user pans/zooms; suppresses follow recentering.
   */
  bool user_moved_view_ = false;

  /** Cursor geo for hover preview; invalid when the mouse left the widget. */
  bool hover_valid_ = false;
  QPointF hover_lat_lon_;

  /** Dragging a plan vertex: kind 0/1/2, or −1 when idle. */
  int drag_plan_kind_ = -1;
  int drag_plan_index_ = -1;

  /** Plan vertex shown in the attribute inspector. −1 when a feature is selected instead. */
  int selected_plan_kind_ = -1;
  int selected_plan_index_ = -1;

  /** Builds the inspector payload for the current selection. */
  MapSelectionInfo currentSelection() const;

  /** Emits @ref selectionChanged when the payload differs from the last one. */
  void publishSelection();

  /** Plan list for kind 0/1/2, or null. */
  QVector<MapPlanPoint>* planPoints(int kind);
  const QVector<MapPlanPoint>* planPoints(int kind) const;

  MapSelectionInfo last_selection_;

  /** Frosted glass tool strip (edit / measure / recenter). */
  MapToolbar* toolbar_ = nullptr;
};

}  // namespace map
}  // namespace autoviz
