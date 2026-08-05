/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QWidget>

#include "autoviz/ui/map/map_layer_store.hpp"
#include "autoviz/ui/map/map_tile_cache.hpp"
#include "autoviz/ui/map/map_types.hpp"

namespace autoviz {
namespace map {

class MapViewportWidget : public QWidget {
  Q_OBJECT

 public:
  explicit MapViewportWidget(QWidget* parent = nullptr);

  void setConfig(const MapPanelConfig& config);
  MapPanelConfig config() const { return config_; }

  void setLayers(const QVector<MapRenderLayerSnapshot>& layers);
  void setFollowTarget(const MapFollowTarget& target);

 signals:
  void viewChanged(double latitude, double longitude, double zoom);

 protected:
  void paintEvent(QPaintEvent* event) override;
  void resizeEvent(QResizeEvent* event) override;
  void wheelEvent(QWheelEvent* event) override;
  void mousePressEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;

 private slots:
  void onTileReady(const MapTileCoord& coord);

 private:
  QPointF centerWorldPixel() const;
  void requestVisibleTiles();
  void drawTiles(QPainter* painter, MapTileCache* cache, const QString& url_template,
                 double opacity);
  void drawLayers(QPainter* painter);
  void drawPoint(QPainter* painter, const MapGeoPoint& point,
                 const MapTopicLayerConfig& style, const QPointF& screen_pos);
  QPointF latLonToScreen(double latitude, double longitude) const;
  QSet<MapTileCoord> visibleTileCoords() const;

  MapPanelConfig config_;
  QVector<MapRenderLayerSnapshot> layers_;
  MapFollowTarget follow_target_;
  MapTileCache base_tile_cache_;
  MapTileCache overlay_tile_cache_;
  bool panning_ = false;
  QPoint last_pan_pos_;
  bool user_moved_view_ = false;
};

}  // namespace map
}  // namespace autoviz
