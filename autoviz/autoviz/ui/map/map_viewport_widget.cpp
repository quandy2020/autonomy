/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/map/map_viewport_widget.hpp"

#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QWheelEvent>

#include <QtMath>

namespace autoviz {
namespace map {
namespace {

constexpr int kTilePixelSize = 256;
constexpr int kMinZoom = 2;
constexpr int kMaxZoom = 20;

void DrawCross(QPainter* painter, const QPointF& center, double half_size) {
  painter->drawLine(QPointF(center.x() - half_size, center.y()),
                    QPointF(center.x() + half_size, center.y()));
  painter->drawLine(QPointF(center.x(), center.y() - half_size),
                    QPointF(center.x(), center.y() + half_size));
}

void DrawDiamond(QPainter* painter, const QPointF& center, double half_size) {
  QPolygonF diamond;
  diamond << QPointF(center.x(), center.y() - half_size)
          << QPointF(center.x() + half_size, center.y())
          << QPointF(center.x(), center.y() + half_size)
          << QPointF(center.x() - half_size, center.y());
  painter->drawPolygon(diamond);
}

void DrawSquare(QPainter* painter, const QPointF& center, double half_size) {
  painter->drawRect(QRectF(center.x() - half_size, center.y() - half_size,
                           half_size * 2.0, half_size * 2.0));
}

void DrawArrow(QPainter* painter, const QPointF& center, double size,
               double heading_deg) {
  painter->save();
  painter->translate(center);
  painter->rotate(-heading_deg);
  QPolygonF arrow;
  arrow << QPointF(0.0, -size)
        << QPointF(size * 0.55, size * 0.75)
        << QPointF(0.0, size * 0.35)
        << QPointF(-size * 0.55, size * 0.75);
  painter->drawPolygon(arrow);
  painter->restore();
}

}  // namespace

MapViewportWidget::MapViewportWidget(QWidget* parent) : QWidget(parent) {
  setMinimumSize(160, 120);
  setMouseTracking(true);
  setFocusPolicy(Qt::StrongFocus);
  base_tile_cache_.setUserAgent(QStringLiteral("Autoviz/1.0 (+https://github.com/openbot)"));
  overlay_tile_cache_.setUserAgent(QStringLiteral("Autoviz/1.0 (+https://github.com/openbot)"));
  connect(&base_tile_cache_, &MapTileCache::tileReady, this,
          &MapViewportWidget::onTileReady);
  connect(&overlay_tile_cache_, &MapTileCache::tileReady, this,
          &MapViewportWidget::onTileReady);
}

void MapViewportWidget::setConfig(const MapPanelConfig& config) {
  const bool base_changed = config_.base_layer != config.base_layer ||
                            config_.custom_tile_url != config.custom_tile_url;
  config_ = config;
  config_.zoom = std::clamp(config_.zoom, static_cast<double>(kMinZoom),
                            static_cast<double>(kMaxZoom));
  if (follow_target_.valid && !config_.follow_channel.isEmpty() &&
      !user_moved_view_) {
    config_.center_latitude = follow_target_.latitude;
    config_.center_longitude = follow_target_.longitude;
  }
  if (base_changed) {
    base_tile_cache_.clear();
  }
  requestVisibleTiles();
  update();
}

void MapViewportWidget::setLayers(const QVector<MapRenderLayerSnapshot>& layers) {
  layers_ = layers;
  update();
}

void MapViewportWidget::setFollowTarget(const MapFollowTarget& target) {
  follow_target_ = target;
  if (target.valid && !config_.follow_channel.isEmpty() && !user_moved_view_) {
    config_.center_latitude = target.latitude;
    config_.center_longitude = target.longitude;
    requestVisibleTiles();
    update();
  }
}

QPointF MapViewportWidget::centerWorldPixel() const {
  return MapProjection::LatLonToWorldPixel(config_.center_latitude,
                                           config_.center_longitude,
                                           static_cast<int>(config_.zoom));
}

QPointF MapViewportWidget::latLonToScreen(double latitude,
                                          double longitude) const {
  const QPointF world = MapProjection::LatLonToWorldPixel(
      latitude, longitude, static_cast<int>(config_.zoom));
  const QPointF center = centerWorldPixel();
  return QPointF(width() * 0.5 + world.x() - center.x(),
                 height() * 0.5 + world.y() - center.y());
}

QSet<MapTileCoord> MapViewportWidget::visibleTileCoords() const {
  QSet<MapTileCoord> coords;
  const QRect range = MapProjection::VisibleTileRange(
      centerWorldPixel(), size(), static_cast<int>(config_.zoom), 1);
  for (int y = range.top(); y < range.top() + range.height(); ++y) {
    for (int x = range.left(); x < range.left() + range.width(); ++x) {
      MapTileCoord coord;
      coord.z = static_cast<int>(config_.zoom);
      coord.x = x;
      coord.y = y;
      coords.insert(coord);
    }
  }
  return coords;
}

void MapViewportWidget::requestVisibleTiles() {
  const QSet<MapTileCoord> coords = visibleTileCoords();
  const QString base_url =
      BaseLayerTileUrlTemplate(config_.base_layer, config_.custom_tile_url);
  base_tile_cache_.requestTiles(base_url, coords);
  for (const MapOverlayLayerConfig& overlay : config_.overlay_layers) {
    if (!overlay.enabled || overlay.tile_url_template.trimmed().isEmpty()) {
      continue;
    }
    overlay_tile_cache_.requestTiles(overlay.tile_url_template, coords);
  }
}

void MapViewportWidget::onTileReady(const MapTileCoord& /*coord*/) { update(); }

void MapViewportWidget::drawTiles(QPainter* painter, MapTileCache* cache,
                                  const QString& url_template, double opacity) {
  if (url_template.trimmed().isEmpty() || cache == nullptr) {
    return;
  }
  const QPointF center = centerWorldPixel();
  const double half_w = width() * 0.5;
  const double half_h = height() * 0.5;
  painter->save();
  painter->setOpacity(std::clamp(opacity, 0.0, 1.0));
  for (const MapTileCoord& coord : visibleTileCoords()) {
    const QImage image = cache->cachedTile(coord);
    if (image.isNull()) {
      continue;
    }
    const double left =
        half_w + coord.x * kTilePixelSize - center.x();
    const double top =
        half_h + coord.y * kTilePixelSize - center.y();
    painter->drawImage(QRectF(left, top, kTilePixelSize, kTilePixelSize), image);
  }
  painter->restore();
}

void MapViewportWidget::drawPoint(QPainter* painter, const MapGeoPoint& point,
                                  const MapTopicLayerConfig& style,
                                  const QPointF& screen_pos) {
  const QColor color = point.color.isValid() ? point.color : style.color.isValid()
                                                               ? style.color
                                                               : QColor(255, 90, 60);
  const double size = std::max(2.0, style.point_size);
  QPen pen(color);
  pen.setWidthF(1.5);
  painter->setPen(pen);
  painter->setBrush(color);

  switch (style.point_style) {
    case MapPointStyle::kDot:
      painter->drawEllipse(screen_pos, size * 0.5, size * 0.5);
      break;
    case MapPointStyle::kArrow:
      if (style.show_heading && !qIsNaN(point.heading_deg)) {
        DrawArrow(painter, screen_pos, size, point.heading_deg);
      } else {
        painter->drawEllipse(screen_pos, size * 0.5, size * 0.5);
      }
      break;
    case MapPointStyle::kDiamond:
      DrawDiamond(painter, screen_pos, size * 0.55);
      break;
    case MapPointStyle::kSquare:
      DrawSquare(painter, screen_pos, size * 0.5);
      break;
    case MapPointStyle::kCross:
      DrawCross(painter, screen_pos, size * 0.55);
      break;
  }

  if (style.show_velocity && !qIsNaN(point.velocity_east_mps) &&
      !qIsNaN(point.velocity_north_mps)) {
    const double speed =
        std::hypot(point.velocity_east_mps, point.velocity_north_mps);
    if (speed > 0.05) {
      const double heading =
          qRadiansToDegrees(std::atan2(point.velocity_east_mps, point.velocity_north_mps));
      const double arrow_len = std::clamp(speed * 2.0, 8.0, 40.0);
      QPen velocity_pen(QColor(255, 255, 255, 200));
      velocity_pen.setWidthF(2.0);
      painter->setPen(velocity_pen);
      const QPointF end(screen_pos.x() + arrow_len * std::sin(qDegreesToRadians(heading)),
                        screen_pos.y() - arrow_len * std::cos(qDegreesToRadians(heading)));
      painter->drawLine(screen_pos, end);
    }
  }
}

void MapViewportWidget::drawLayers(QPainter* painter) {
  for (const MapRenderLayerSnapshot& layer : layers_) {
    painter->save();
    painter->setOpacity(std::clamp(layer.style.layer_opacity, 0.0, 1.0));

    for (const MapGeoPolygon& polygon : layer.polygons) {
      if (polygon.outer_ring.size() < 3) {
        continue;
      }
      QPolygonF screen_ring;
      screen_ring.reserve(polygon.outer_ring.size());
      for (const QPointF& lat_lon : polygon.outer_ring) {
        screen_ring.push_back(latLonToScreen(lat_lon.y(), lat_lon.x()));
      }
      QPen outline(polygon.outline_color);
      outline.setWidthF(polygon.outline_width);
      painter->setPen(outline);
      painter->setBrush(polygon.fill_color);
      painter->drawPolygon(screen_ring);
    }

    for (const MapGeoLine& line : layer.lines) {
      if (line.lat_lon_points.size() < 2) {
        continue;
      }
      QPen pen(line.color);
      pen.setWidthF(line.width);
      painter->setPen(pen);
      QPainterPath path;
      path.moveTo(latLonToScreen(line.lat_lon_points.front().y(),
                                 line.lat_lon_points.front().x()));
      for (int i = 1; i < line.lat_lon_points.size(); ++i) {
        path.lineTo(latLonToScreen(line.lat_lon_points.at(i).y(),
                                   line.lat_lon_points.at(i).x()));
      }
      painter->drawPath(path);
    }

    if (layer.style.time_range != MapTimeRange::kLatest &&
        layer.points.size() >= 2) {
      QPen trail_pen(layer.style.color.isValid() ? layer.style.color
                                                   : QColor(255, 90, 60, 180));
      trail_pen.setWidthF(2.0);
      painter->setPen(trail_pen);
      QPainterPath trail;
      trail.moveTo(latLonToScreen(layer.points.front().latitude,
                                  layer.points.front().longitude));
      for (int i = 1; i < layer.points.size(); ++i) {
        trail.lineTo(latLonToScreen(layer.points.at(i).latitude,
                                    layer.points.at(i).longitude));
      }
      painter->drawPath(trail);
    }

    for (const MapGeoPoint& point : layer.points) {
      drawPoint(painter, point, layer.style,
                latLonToScreen(point.latitude, point.longitude));
    }
    painter->restore();
  }
}

void MapViewportWidget::paintEvent(QPaintEvent* /*event*/) {
  QPainter painter(this);
  painter.fillRect(rect(), QColor(30, 34, 40));

  requestVisibleTiles();
  const QString base_url =
      BaseLayerTileUrlTemplate(config_.base_layer, config_.custom_tile_url);
  drawTiles(&painter, &base_tile_cache_, base_url, 1.0);
  for (const MapOverlayLayerConfig& overlay : config_.overlay_layers) {
    if (!overlay.enabled) {
      continue;
    }
    drawTiles(&painter, &overlay_tile_cache_, overlay.tile_url_template,
              overlay.opacity);
  }
  drawLayers(&painter);

  painter.setPen(QColor(180, 180, 190));
  painter.drawText(QRect(8, 8, width() - 16, 20), Qt::AlignLeft | Qt::AlignVCenter,
                   tr("Zoom %1").arg(QString::number(config_.zoom, 'f', 1)));
}

void MapViewportWidget::resizeEvent(QResizeEvent* event) {
  QWidget::resizeEvent(event);
  requestVisibleTiles();
}

void MapViewportWidget::wheelEvent(QWheelEvent* event) {
  if (event == nullptr) {
    return;
  }
  const double delta = event->angleDelta().y() > 0 ? 0.25 : -0.25;
  config_.zoom = std::clamp(config_.zoom + delta, static_cast<double>(kMinZoom),
                            static_cast<double>(kMaxZoom));
  const QPointF center = centerWorldPixel();
  const QPointF cursor_world(
      center.x() + (event->position().x() - width() * 0.5),
      center.y() + (event->position().y() - height() * 0.5));
  const QPointF new_center = MapProjection::WorldPixelToLatLon(cursor_world,
                                                               static_cast<int>(config_.zoom));
  config_.center_latitude = new_center.x();
  config_.center_longitude = new_center.y();
  user_moved_view_ = true;
  requestVisibleTiles();
  emit viewChanged(config_.center_latitude, config_.center_longitude, config_.zoom);
  update();
  event->accept();
}

void MapViewportWidget::mousePressEvent(QMouseEvent* event) {
  if (event != nullptr && event->button() == Qt::LeftButton) {
    panning_ = true;
    last_pan_pos_ = event->pos();
    user_moved_view_ = true;
    event->accept();
  }
}

void MapViewportWidget::mouseMoveEvent(QMouseEvent* event) {
  if (!panning_ || event == nullptr) {
    return;
  }
  const QPoint delta = event->pos() - last_pan_pos_;
  last_pan_pos_ = event->pos();
  const QPointF center = centerWorldPixel();
  const QPointF shifted(center.x() - delta.x(), center.y() - delta.y());
  const QPointF lat_lon =
      MapProjection::WorldPixelToLatLon(shifted, static_cast<int>(config_.zoom));
  config_.center_latitude = lat_lon.x();
  config_.center_longitude = lat_lon.y();
  requestVisibleTiles();
  emit viewChanged(config_.center_latitude, config_.center_longitude, config_.zoom);
  update();
  event->accept();
}

void MapViewportWidget::mouseReleaseEvent(QMouseEvent* event) {
  if (event != nullptr && event->button() == Qt::LeftButton) {
    panning_ = false;
    event->accept();
  }
}

}  // namespace map
}  // namespace autoviz
