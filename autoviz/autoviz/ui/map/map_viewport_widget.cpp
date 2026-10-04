/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/map/map_viewport_widget.hpp"

#include <QClipboard>
#include <QHash>
#include <QFileDialog>
#include <QFontMetrics>
#include <QGestureEvent>
#include <QGuiApplication>
#include <QKeyEvent>
#include <QLineF>
#include <QMenu>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPinchGesture>
#include <QSet>
#include <QStringList>
#include <QTimer>
#include <QWheelEvent>
#include <QtGlobal>

#include <QtMath>

#include <algorithm>
#include <cmath>

#include "autoviz/ui/map/map_toolbar.hpp"
#include "autoviz/ui/theme/glass.hpp"

namespace autoviz {
namespace map {
namespace {

constexpr int kTilePixelSize = 256;
constexpr int kMinZoom = 2;
constexpr int kMaxZoom = 20;
/** Imagery zoom. One step above the camera so a 2x display is not upscaled. */
constexpr int kMaxTileZoom = 22;

int Channel(int value, int neighbor_sum, int amount) {
  const int average = neighbor_sum / 4;
  return std::clamp(value + (amount * (value - average)) / 256, 0, 255);
}

QImage SharpenGlyphs(const QImage& input, int amount) {
  const QImage src = input.convertToFormat(QImage::Format_ARGB32);
  if (src.width() < 3 || src.height() < 3) {
    return src;
  }
  QImage dst(src.size(), QImage::Format_ARGB32);
  const int width = src.width();
  const int height = src.height();
  for (int y = 0; y < height; ++y) {
    const auto* row = reinterpret_cast<const QRgb*>(src.constScanLine(y));
    const auto* up =
        reinterpret_cast<const QRgb*>(src.constScanLine(std::max(y - 1, 0)));
    const auto* down = reinterpret_cast<const QRgb*>(
        src.constScanLine(std::min(y + 1, height - 1)));
    auto* out = reinterpret_cast<QRgb*>(dst.scanLine(y));
    for (int x = 0; x < width; ++x) {
      const int left = std::max(x - 1, 0);
      const int right = std::min(x + 1, width - 1);
      const QRgb pixel = row[x];
      const int neighbor_red =
          qRed(up[x]) + qRed(down[x]) + qRed(row[left]) + qRed(row[right]);
      const int neighbor_green = qGreen(up[x]) + qGreen(down[x]) +
                                 qGreen(row[left]) + qGreen(row[right]);
      const int neighbor_blue =
          qBlue(up[x]) + qBlue(down[x]) + qBlue(row[left]) + qBlue(row[right]);
      out[x] = qRgba(Channel(qRed(pixel), neighbor_red, amount),
                     Channel(qGreen(pixel), neighbor_green, amount),
                     Channel(qBlue(pixel), neighbor_blue, amount), qAlpha(pixel));
    }
  }
  return dst;
}

QImage PrepareTileBitmap(const QImage& source, int target_w, int target_h) {
  static QHash<QString, QImage> cache;
  const QString key = QString::number(source.cacheKey()) + QLatin1Char('/') +
                      QString::number(target_w) + QLatin1Char('x') +
                      QString::number(target_h);
  if (const auto found = cache.constFind(key); found != cache.cend()) {
    return found.value();
  }
  if (cache.size() > 80) {
    cache.clear();
  }
  QImage prepared = source;
  if (source.width() != target_w || source.height() != target_h) {
    prepared = source.scaled(target_w, target_h, Qt::IgnoreAspectRatio,
                             Qt::SmoothTransformation);
    const double growth =
        static_cast<double>(target_w) / static_cast<double>(std::max(source.width(), 1));
    if (growth >= 1.12) {
      const int amount = growth >= 1.6 ? 170 : 110;
      prepared = SharpenGlyphs(prepared, amount);
    }
  }
  cache.insert(key, prepared);
  return prepared;
}

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

void DrawFeatureLabel(QPainter* painter, const QPointF& origin, const QString& label) {
  if (painter == nullptr || label.isEmpty()) {
    return;
  }
  QFont font = painter->font();
  font.setPixelSize(11);
  font.setBold(true);
  painter->setFont(font);
  painter->setPen(QColor(255, 255, 255, 220));
  painter->drawText(origin + QPointF(1.0, 1.0), label);
  painter->setPen(QColor(15, 23, 42));
  painter->drawText(origin, label);
}

constexpr int kScaleLengthsMeters[] = {
    5,     10,    25,     50,     100,    150,     250,     500,
    1000,  2000,  5000,   10000,  20000,  50000,   100000,  200000,
    500000, 1000000, 2000000};

constexpr int kScaleLengthsFeet[] = {
    10,   25,   50,    100,   250,    500,     1000,    3000,
    6000, 10000, 25000, 50000, 100000, 250000,  500000,  1000000,
    3000000};

QString FormatScaleMeters(int meters) {
  if (meters >= 1000) {
    if (meters >= 100000) {
      return QString::number(meters / 1000) + QObject::tr(" km");
    }
    const int tenths = static_cast<int>(std::lround(meters / 100.0));
    return QString::number(tenths / 10.0, 'f', tenths % 10 == 0 ? 0 : 1) +
           QObject::tr(" km");
  }
  return QString::number(meters) + QObject::tr(" m");
}

QString FormatScaleFeet(int feet) {
  if (feet >= 5280) {
    const double miles = static_cast<double>(feet) / 5280.0;
    return QString::number(miles, 'f', miles >= 10.0 ? 0 : 1) +
           QObject::tr(" mi");
  }
  return QString::number(feet) + QObject::tr(" ft");
}

int ChooseScaleLength(double probe, const int* lengths, int count) {
  int chosen = lengths[count - 1];
  for (int i = 0; i < count - 1; ++i) {
    const double mid = (lengths[i] + lengths[i + 1]) * 0.5;
    if (probe < mid) {
      chosen = lengths[i];
      break;
    }
  }
  return chosen;
}

bool IsRallyLabel(const QString& label) {
  return label.contains(QStringLiteral("rally"), Qt::CaseInsensitive);
}

double HaversineMeters(double lat1, double lon1, double lat2, double lon2) {
  constexpr double kEarthRadius = 6378137.0;
  const double d_lat = qDegreesToRadians(lat2 - lat1);
  const double d_lon = qDegreesToRadians(lon2 - lon1);
  const double a = std::sin(d_lat * 0.5) * std::sin(d_lat * 0.5) +
                   std::cos(qDegreesToRadians(lat1)) *
                       std::cos(qDegreesToRadians(lat2)) *
                       std::sin(d_lon * 0.5) * std::sin(d_lon * 0.5);
  return 2.0 * kEarthRadius * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
}

}  // namespace

MapViewportWidget::MapViewportWidget(QWidget* parent) : QWidget(parent) {
  setMinimumSize(160, 120);
  setMouseTracking(true);
  setFocusPolicy(Qt::StrongFocus);
  setAttribute(Qt::WA_StyledBackground, true);
  base_tile_cache_.setUserAgent(QStringLiteral("Autoviz/1.0 (+https://github.com/openbot)"));
  overlay_tile_cache_.setUserAgent(QStringLiteral("Autoviz/1.0 (+https://github.com/openbot)"));
  connect(&base_tile_cache_, &MapTileCache::tileReady, this,
          &MapViewportWidget::onTileReady);
  connect(&base_tile_cache_, &MapTileCache::tileUnavailable, this,
          &MapViewportWidget::onTileUnavailable);
  connect(&overlay_tile_cache_, &MapTileCache::tileReady, this,
          &MapViewportWidget::onTileReady);

  toolbar_ = new MapToolbar(this);
  MapToolbarCallbacks callbacks;
  callbacks.on_edit_tool = [this](MapEditTool tool) {
    config_.edit_tool = tool;
    if (tool != MapEditTool::kPan) {
      geo_map_.setMeasuring(false);
    }
    syncToolbar();
    syncCursor();
    emit planChanged();
    update();
  };
  callbacks.on_measure = [this]() {
    geo_map_.setMeasuring(!geo_map_.measuring());
    if (geo_map_.measuring()) {
      config_.edit_tool = MapEditTool::kPan;
    }
    syncToolbar();
    syncCursor();
    publishMapStatus();
    update();
  };
  callbacks.on_recenter = [this]() { recenter(); };
  toolbar_->setCallbacks(callbacks);
  positionToolbar();
  syncToolbar();
  syncCursor();

  setAttribute(Qt::WA_AcceptTouchEvents);
  grabGesture(Qt::PinchGesture);
  hold_timer_ = new QTimer(this);
  hold_timer_->setSingleShot(true);
  hold_timer_->setInterval(600);
  connect(hold_timer_, &QTimer::timeout, this, [this]() {
    if (!panning_) {
      showMapMenu(hold_press_pos_);
    }
  });
}

void MapViewportWidget::setConfig(const MapPanelConfig& config) {
  const bool base_changed = config_.base_layer != config.base_layer ||
                            config_.custom_tile_url != config.custom_tile_url;
  const double pan_latitude = config_.center_latitude;
  const double pan_longitude = config_.center_longitude;
  const double pan_zoom = config_.zoom;
  const bool keep_camera = panning_;
  config_ = config;
  config_.zoom = std::clamp(config_.zoom, static_cast<double>(kMinZoom),
                            static_cast<double>(kMaxZoom));
  geo_map_.syncSources(config_.geojson_sources);
  publishMapStatus();
  if (follow_target_.valid && !config_.follow_channel.isEmpty() &&
      !user_moved_view_) {
    config_.center_latitude = follow_target_.latitude;
    config_.center_longitude = follow_target_.longitude;
  }
  if (keep_camera) {
    config_.center_latitude = pan_latitude;
    config_.center_longitude = pan_longitude;
    config_.zoom = pan_zoom;
  }
  syncToolbar();
  syncCursor();
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

void MapViewportWidget::setGcsTarget(const MapFollowTarget& target) {
  gcs_target_ = target;
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
                                           tileZoom()) *
         tileScale();
}

int MapViewportWidget::tileZoom() const {
  const double ratio = std::max(devicePixelRatioF(), 1.0);
  const double needed = config_.zoom + std::log2(ratio);
  const int zoom = static_cast<int>(std::lround(needed));
  const int native_max = std::min(kMaxTileZoom, BaseLayerMaxNativeZoom(config_.base_layer));
  return std::clamp(zoom, kMinZoom, native_max);
}

double MapViewportWidget::tileScale() const {
  return std::pow(2.0, config_.zoom - static_cast<double>(tileZoom()));
}

QPointF MapViewportWidget::screenToLatLon(const QPointF& screen) const {
  const QPointF center = centerWorldPixel();
  const QPointF world(center.x() + (screen.x() - width() * 0.5),
                      center.y() + (screen.y() - height() * 0.5));
  const double scale = std::max(tileScale(), 1e-6);
  return MapProjection::WorldPixelToLatLon(world / scale, tileZoom());
}

void MapViewportWidget::alignLatLonToScreen(double latitude, double longitude,
                                            const QPointF& screen) {
  const double scale = tileScale();
  const QPointF anchor =
      MapProjection::LatLonToWorldPixel(latitude, longitude, tileZoom()) * scale;
  const QPointF center_world(anchor.x() - (screen.x() - width() * 0.5),
                             anchor.y() - (screen.y() - height() * 0.5));
  const QPointF lat_lon =
      MapProjection::WorldPixelToLatLon(center_world / std::max(scale, 1e-6),
                                        tileZoom());
  config_.center_latitude = lat_lon.x();
  config_.center_longitude = lat_lon.y();
}

QPointF MapViewportWidget::latLonToScreen(double latitude,
                                          double longitude) const {
  const QPointF world =
      MapProjection::LatLonToWorldPixel(latitude, longitude, tileZoom()) *
      tileScale();
  const QPointF center = centerWorldPixel();
  return QPointF(width() * 0.5 + world.x() - center.x(),
                 height() * 0.5 + world.y() - center.y());
}

QVector<MapViewportWidget::TilePlacement> MapViewportWidget::visibleTilePlacements()
    const {
  QVector<TilePlacement> placed;
  const int zoom = tileZoom();
  const double scale = std::max(tileScale(), 1e-6);
  const QPointF center = MapProjection::LatLonToWorldPixel(
      config_.center_latitude, config_.center_longitude, zoom);
  const int tile_count = std::max(1, MapProjection::TileCount(zoom));
  const double half_w = width() / scale * 0.5;
  const double half_h = height() / scale * 0.5;
  constexpr int kMargin = 1;
  const int min_x =
      static_cast<int>(std::floor((center.x() - half_w) / kTilePixelSize)) - kMargin;
  const int max_x =
      static_cast<int>(std::floor((center.x() + half_w) / kTilePixelSize)) + kMargin;
  const int min_y = std::clamp(
      static_cast<int>(std::floor((center.y() - half_h) / kTilePixelSize)) - kMargin, 0,
      tile_count - 1);
  const int max_y = std::clamp(
      static_cast<int>(std::floor((center.y() + half_h) / kTilePixelSize)) + kMargin, 0,
      tile_count - 1);
  for (int row = min_y; row <= max_y; ++row) {
    for (int column = min_x; column <= max_x; ++column) {
      TilePlacement tile;
      tile.column = column;
      tile.row = row;
      tile.coord.z = zoom;
      tile.coord.y = row;
      tile.coord.x = column % tile_count;
      if (tile.coord.x < 0) {
        tile.coord.x += tile_count;
      }
      placed.push_back(tile);
    }
  }
  return placed;
}

void MapViewportWidget::requestVisibleTiles() {
  QSet<MapTileCoord> coords;
  for (const TilePlacement& tile : visibleTilePlacements()) {
    coords.insert(tile.coord);
    MapTileCoord parent = tile.coord;
    for (int level = 0; level < 2 && parent.z > kMinZoom; ++level) {
      parent.z -= 1;
      parent.x /= 2;
      parent.y /= 2;
      coords.insert(parent);
    }
  }
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

void MapViewportWidget::onTileReady(const MapTileCoord& coord) {
  noteDownloadFinished(coord);
  update();
}

void MapViewportWidget::onTileUnavailable(const MapTileCoord& coord) {
  noteDownloadFinished(coord);
}

void MapViewportWidget::noteDownloadFinished(const MapTileCoord& coord) {
  if (!download_pending_.remove(coord) || download_total_ <= 0) {
    return;
  }
  const int done = download_total_ - download_pending_.size();
  emit mapStatusChanged(tr("Offline tiles %1/%2").arg(done).arg(download_total_));
  if (download_pending_.isEmpty()) {
    download_total_ = 0;
  }
}

void MapViewportWidget::downloadVisibleTiles(int min_zoom, int max_zoom) {
  min_zoom = std::clamp(min_zoom, kMinZoom, kMaxZoom);
  max_zoom = std::clamp(max_zoom, min_zoom, kMaxZoom);
  const QPointF corners[4] = {
      screenToLatLon(QPointF(0, 0)),
      screenToLatLon(QPointF(width(), 0)),
      screenToLatLon(QPointF(0, height())),
      screenToLatLon(QPointF(width(), height())),
  };
  QSet<MapTileCoord> coords;
  for (int zoom = max_zoom; zoom >= min_zoom; --zoom) {
    int min_x = 0;
    int max_x = 0;
    int min_y = 0;
    int max_y = 0;
    const int tile_count = std::max(1, MapProjection::TileCount(zoom));
    for (int i = 0; i < 4; ++i) {
      const MapTileCoord tile =
          MapProjection::LatLonToTile(corners[i].x(), corners[i].y(), zoom);
      if (i == 0) {
        min_x = max_x = tile.x;
        min_y = max_y = tile.y;
        continue;
      }
      min_x = std::min(min_x, tile.x);
      max_x = std::max(max_x, tile.x);
      min_y = std::min(min_y, tile.y);
      max_y = std::max(max_y, tile.y);
    }
    if (max_x - min_x > tile_count / 2) {
      continue;
    }
    for (int y = min_y; y <= max_y; ++y) {
      for (int x = min_x; x <= max_x; ++x) {
        MapTileCoord coord;
        coord.z = zoom;
        coord.x = x;
        coord.y = y;
        if (base_tile_cache_.cachedTile(coord).isNull() &&
            !base_tile_cache_.isUnavailable(coord)) {
          coords.insert(coord);
        }
        if (coords.size() >= 400) {
          break;
        }
      }
      if (coords.size() >= 400) {
        break;
      }
    }
    if (coords.size() >= 400) {
      break;
    }
  }
  download_pending_ = coords;
  download_total_ = coords.size();
  if (coords.isEmpty()) {
    emit mapStatusChanged(tr("Offline tiles already cached"));
    return;
  }
  const QString base_url =
      BaseLayerTileUrlTemplate(config_.base_layer, config_.custom_tile_url);
  base_tile_cache_.requestTiles(base_url, coords);
  emit mapStatusChanged(tr("Offline tiles 0/%1").arg(download_total_));
}

void MapViewportWidget::drawTiles(QPainter* painter, MapTileCache* cache,
                                  const QString& url_template, double opacity) {
  if (url_template.trimmed().isEmpty() || cache == nullptr) {
    return;
  }
  const QPointF center = centerWorldPixel();
  const double half_w = width() * 0.5;
  const double half_h = height() * 0.5;
  const double tile_px = kTilePixelSize * tileScale();
  const double dpr = std::max(
      painter->device() != nullptr ? painter->device()->devicePixelRatioF() : 1.0,
      1.0);
  const auto snap = [dpr](double value) { return std::round(value * dpr) / dpr; };
  painter->save();
  painter->setOpacity(std::clamp(opacity, 0.0, 1.0));
  for (const TilePlacement& tile : visibleTilePlacements()) {
    QImage image = cache->cachedTile(tile.coord);
    int ancestor_level = 0;
    MapTileCoord ancestor = tile.coord;
    while (image.isNull() && ancestor.z > kMinZoom) {
      ancestor.x /= 2;
      ancestor.y /= 2;
      ancestor.z -= 1;
      ++ancestor_level;
      image = cache->cachedTile(ancestor);
    }
    if (image.isNull()) {
      continue;
    }
    const double left = snap(half_w + tile.column * tile_px - center.x());
    const double top = snap(half_h + tile.row * tile_px - center.y());
    const int target_w =
        std::max(1, static_cast<int>(std::lround(tile_px * dpr)));
    const int target_h = target_w;
    QImage source = image;
    if (ancestor_level > 0) {
      const int span = 1 << ancestor_level;
      const int src_w = std::max(image.width() / span, 1);
      const int src_h = std::max(image.height() / span, 1);
      const int sub_x = tile.coord.x & (span - 1);
      const int sub_y = tile.coord.y & (span - 1);
      source = image.copy(QRect(sub_x * src_w, sub_y * src_h, src_w, src_h));
    }
    source = PrepareTileBitmap(source, target_w, target_h);
    source.detach();
    source.setDevicePixelRatio(dpr);
    painter->setRenderHint(QPainter::SmoothPixmapTransform, false);
    painter->drawImage(QPointF(left, top), source);
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

  if (!point.label.isEmpty()) {
    DrawFeatureLabel(painter, screen_pos + QPointF(size * 0.6 + 4.0, 4.0), point.label);
  }
}

void MapViewportWidget::drawLayers(QPainter* painter) {
  QVector<MapRenderLayerSnapshot> all = layers_;
  all += geo_map_.renderLayers();
  const GeoMapHit& selected = geo_map_.selection();
  for (const MapRenderLayerSnapshot& layer : all) {
    painter->save();
    painter->setOpacity(std::clamp(layer.style.layer_opacity, 0.0, 1.0));

    for (int polygon_index = 0; polygon_index < layer.polygons.size(); ++polygon_index) {
      const MapGeoPolygon& polygon = layer.polygons.at(polygon_index);
      if (polygon.outer_ring.size() < 3) {
        continue;
      }
      QPolygonF screen_ring;
      screen_ring.reserve(polygon.outer_ring.size());
      for (const QPointF& lat_lon : polygon.outer_ring) {
        screen_ring.push_back(latLonToScreen(lat_lon.y(), lat_lon.x()));
      }
      QPen outline(polygon.outline_color);
      const bool selected_polygon = selected.valid && selected.layer_id == layer.channel &&
                                    selected.kind == GeoMapFeatureKind::kPolygon &&
                                    selected.index == polygon_index;
      outline.setWidthF(std::max(polygon.outline_width, selected_polygon ? 3.0f : 2.0f));
      outline.setStyle(selected_polygon ? Qt::SolidLine : Qt::DashLine);
      if (selected_polygon) {
        outline.setColor(QColor(234, 179, 8));
      }
      painter->setPen(outline);
      painter->setBrush(polygon.fill_color);
      painter->drawPolygon(screen_ring);
      if (!polygon.label.isEmpty() && !screen_ring.isEmpty()) {
        QPointF centroid(0.0, 0.0);
        for (const QPointF& vertex : screen_ring) {
          centroid += vertex;
        }
        centroid /= static_cast<double>(screen_ring.size());
        DrawFeatureLabel(painter, centroid, polygon.label);
      }
    }

    for (int line_index = 0; line_index < layer.lines.size(); ++line_index) {
      const MapGeoLine& line = layer.lines.at(line_index);
      if (line.lat_lon_points.size() < 2) {
        continue;
      }
      const bool selected_line = selected.valid && selected.layer_id == layer.channel &&
                                 selected.kind == GeoMapFeatureKind::kLine &&
                                 selected.index == line_index;
      QPen pen(selected_line ? QColor(234, 179, 8) : line.color);
      pen.setWidthF(selected_line ? std::max(line.width, 4.0f) : line.width);
      painter->setPen(pen);
      QPainterPath path;
      path.moveTo(latLonToScreen(line.lat_lon_points.front().y(),
                                 line.lat_lon_points.front().x()));
      for (int i = 1; i < line.lat_lon_points.size(); ++i) {
        path.lineTo(latLonToScreen(line.lat_lon_points.at(i).y(),
                                   line.lat_lon_points.at(i).x()));
      }
      painter->drawPath(path);
      if (!line.label.isEmpty()) {
        const QPointF from_screen = latLonToScreen(
            line.lat_lon_points.at(line.lat_lon_points.size() / 2).y(),
            line.lat_lon_points.at(line.lat_lon_points.size() / 2).x());
        DrawFeatureLabel(painter, from_screen + QPointF(6.0, -6.0), line.label);
      }
      const QPointF from = latLonToScreen(
          line.lat_lon_points.at(line.lat_lon_points.size() - 2).y(),
          line.lat_lon_points.at(line.lat_lon_points.size() - 2).x());
      const QPointF to = latLonToScreen(line.lat_lon_points.back().y(),
                                       line.lat_lon_points.back().x());
      drawMissionArrow(painter, from, to, line.color);
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

    for (int point_index = 0; point_index < layer.points.size(); ++point_index) {
      const MapGeoPoint& point = layer.points.at(point_index);
      const QPointF screen = latLonToScreen(point.latitude, point.longitude);
      const bool selected_point = selected.valid && selected.layer_id == layer.channel &&
                                  selected.kind == GeoMapFeatureKind::kPoint &&
                                  selected.index == point_index;
      if (selected_point) {
        painter->setPen(QPen(QColor(234, 179, 8), 2.0));
        painter->setBrush(Qt::NoBrush);
        painter->drawEllipse(screen, layer.style.point_size + 5.0,
                             layer.style.point_size + 5.0);
      }
      if (IsRallyLabel(point.label)) {
        painter->setPen(QPen(point.color.isValid() ? point.color
                                                    : layer.style.color,
                             2.0));
        painter->setBrush(Qt::NoBrush);
        painter->drawEllipse(screen, layer.style.point_size + 6.0,
                             layer.style.point_size + 6.0);
      }
      drawPoint(painter, point, layer.style, screen);
    }
    painter->restore();
  }
}

void MapViewportWidget::paintEvent(QPaintEvent* /*event*/) {
  QPainter painter(this);
  const glass::OverlayTokens overlay = glass::Overlay();
  painter.fillRect(rect(), QColor(overlay.fill.red(), overlay.fill.green(),
                                  overlay.fill.blue(), 255));

  requestVisibleTiles();
  const QString base_url =
      BaseLayerTileUrlTemplate(config_.base_layer, config_.custom_tile_url);
  drawTiles(&painter, &base_tile_cache_, base_url, 1.0);
  for (const MapOverlayLayerConfig& overlay_layer : config_.overlay_layers) {
    if (!overlay_layer.enabled) {
      continue;
    }
    drawTiles(&painter, &overlay_tile_cache_, overlay_layer.tile_url_template,
              overlay_layer.opacity);
  }
  drawLayers(&painter);
  drawMeasure(&painter);
  drawVehicle(&painter);
  drawGcs(&painter);
  drawPlan(&painter);
  drawHoverPreview(&painter);
  drawCompass(&painter);
  drawAttribution(&painter);
  drawScaleBar(&painter);
  drawAttributeCard(&painter);
}

void MapViewportWidget::drawScaleBar(QPainter* painter) const {
  constexpr double kProbePixels = 100.0;
  const QPointF origin(18.0, height() - 30.0);
  const QPointF left_ll = screenToLatLon(origin);
  const QPointF right_ll = screenToLatLon(origin + QPointF(kProbePixels, 0.0));
  const double probe_meters = HaversineMeters(left_ll.x(), left_ll.y(),
                                              right_ll.x(), right_ll.y());
  if (!(probe_meters > 1.0)) {
    return;
  }

  const bool use_feet = config_.distance_unit == MapDistanceUnit::kFeet;
  const double probe = use_feet ? probe_meters * 3.2808399 : probe_meters;
  const int chosen =
      use_feet ? ChooseScaleLength(probe, kScaleLengthsFeet,
                                   static_cast<int>(std::size(kScaleLengthsFeet)))
               : ChooseScaleLength(probe, kScaleLengthsMeters,
                                   static_cast<int>(std::size(kScaleLengthsMeters)));
  const double bar_width =
      std::clamp(kProbePixels * (static_cast<double>(chosen) / probe), 36.0,
                 width() * 0.45);
  const QString label =
      use_feet ? FormatScaleFeet(chosen) : FormatScaleMeters(chosen);
  const glass::OverlayTokens g = glass::Overlay();

  painter->save();
  painter->setRenderHint(QPainter::Antialiasing, true);
  QFont font = painter->font();
  font.setPixelSize(11);
  font.setBold(true);
  painter->setFont(font);
  const QFontMetrics metrics(font);
  const QRectF card(origin.x() - 8.0, origin.y() - metrics.height() - 14.0,
                    bar_width + 20.0, metrics.height() + 24.0);
  QPainterPath path;
  path.addRoundedRect(card, 10.0, 10.0);
  painter->fillPath(path, g.fill_deep);
  painter->setPen(QPen(QColor(g.rim.red(), g.rim.green(), g.rim.blue(), 70), 1.0));
  painter->drawPath(path);

  painter->setPen(QColor(g.value.red(), g.value.green(), g.value.blue(), 230));
  painter->drawText(QRectF(card.left() + 8.0, card.top() + 4.0, card.width() - 12.0,
                           metrics.height()),
                    Qt::AlignLeft | Qt::AlignVCenter, label);

  const double x = origin.x();
  const double y = origin.y();
  QPen bar(QColor(g.accent.red(), g.accent.green(), g.accent.blue(), 220));
  bar.setWidth(2);
  painter->setPen(bar);
  painter->drawLine(QPointF(x, y), QPointF(x + bar_width, y));
  painter->drawLine(QPointF(x, y - 5), QPointF(x, y + 1));
  painter->drawLine(QPointF(x + bar_width, y - 5), QPointF(x + bar_width, y + 1));
  painter->restore();
}

void MapViewportWidget::resizeEvent(QResizeEvent* event) {
  QWidget::resizeEvent(event);
  positionToolbar();
  requestVisibleTiles();
}

void MapViewportWidget::wheelEvent(QWheelEvent* event) {
  if (event == nullptr) {
    return;
  }
  const QPointF cursor = event->position();
  const QPointF anchor = screenToLatLon(cursor);
  const double steps = event->angleDelta().y() / 120.0;
  if (qFuzzyIsNull(steps)) {
    return;
  }
  config_.zoom = std::clamp(config_.zoom + steps, static_cast<double>(kMinZoom),
                            static_cast<double>(kMaxZoom));
  alignLatLonToScreen(anchor.x(), anchor.y(), cursor);
  user_moved_view_ = true;
  requestVisibleTiles();
  emit viewChanged(config_.center_latitude, config_.center_longitude, config_.zoom);
  update();
  event->accept();
}

void MapViewportWidget::mousePressEvent(QMouseEvent* event) {
  if (event == nullptr) {
    return;
  }
  setFocus(Qt::MouseFocusReason);
  if (event->button() == Qt::RightButton) {
    if (hold_timer_ != nullptr) {
      hold_timer_->stop();
    }
    showMapMenu(event->pos());
    event->accept();
    return;
  }
  if (event->button() == Qt::LeftButton) {
    if (config_.edit_tool == MapEditTool::kPan && !geo_map_.measuring()) {
      int kind = -1;
      int index = -1;
      if (hitPlanVertex(event->position(), 10.0, &kind, &index)) {
        drag_plan_kind_ = kind;
        drag_plan_index_ = index;
        selected_plan_kind_ = kind;
        selected_plan_index_ = index;
        geo_map_.clearSelection();
        publishSelection();
        update();
        if (hold_timer_ != nullptr) {
          hold_timer_->stop();
        }
        grabMouse();
        setCursor(Qt::ClosedHandCursor);
        event->accept();
        return;
      }
    }
    pan_anchor_ = screenToLatLon(event->position());
    armPressAndHold(event->position().toPoint());
    event->accept();
  }
}

void MapViewportWidget::mouseMoveEvent(QMouseEvent* event) {
  if (event == nullptr) {
    return;
  }

  const QPointF lat_lon = screenToLatLon(event->position());
  hover_valid_ = true;
  hover_lat_lon_ = lat_lon;
  emit cursorGeoChanged(lat_lon.x(), lat_lon.y());

  if (drag_plan_kind_ >= 0 && (event->buttons() & Qt::LeftButton)) {
    QVector<MapPlanPoint>* points = nullptr;
    if (drag_plan_kind_ == 0) {
      points = &config_.waypoints;
    } else if (drag_plan_kind_ == 1) {
      points = &config_.geofence;
    } else if (drag_plan_kind_ == 2) {
      points = &config_.rally_points;
    }
    if (points != nullptr && drag_plan_index_ >= 0 &&
        drag_plan_index_ < points->size()) {
      (*points)[drag_plan_index_] = MapPlanPoint{lat_lon.x(), lat_lon.y()};
      publishSelection();
      update();
    }
    event->accept();
    return;
  }

  if (!(event->buttons() & Qt::LeftButton)) {
    if (config_.edit_tool != MapEditTool::kPan || geo_map_.measuring()) {
      update();
    } else {
      int kind = -1;
      int index = -1;
      syncCursor();
      if (hitPlanVertex(event->position(), 10.0, &kind, &index)) {
        setCursor(Qt::SizeAllCursor);
      }
    }
    return;
  }

  if (!panning_) {
    if ((event->position().toPoint() - hold_press_pos_).manhattanLength() < 6) {
      return;
    }
    if (hold_timer_ != nullptr) {
      hold_timer_->stop();
    }
    panning_ = true;
    user_moved_view_ = true;
    grabMouse();
    syncCursor();
  }
  alignLatLonToScreen(pan_anchor_.x(), pan_anchor_.y(), event->position());
  requestVisibleTiles();
  emit viewChanged(config_.center_latitude, config_.center_longitude, config_.zoom);
  update();
  event->accept();
}

void MapViewportWidget::mouseReleaseEvent(QMouseEvent* event) {
  if (hold_timer_ != nullptr) {
    hold_timer_->stop();
  }
  if (event != nullptr && event->button() == Qt::LeftButton) {
    if (drag_plan_kind_ >= 0) {
      drag_plan_kind_ = -1;
      drag_plan_index_ = -1;
      releaseMouse();
      syncCursor();
      emit planChanged();
      publishSelection();
      event->accept();
      return;
    }
    const bool was_pan = panning_;
    panning_ = false;
    if (was_pan) {
      releaseMouse();
      syncCursor();
      emit viewChanged(config_.center_latitude, config_.center_longitude,
                       config_.zoom);
    } else {
      handleMapClick(event->pos());
    }
    event->accept();
  }
}

void MapViewportWidget::leaveEvent(QEvent* event) {
  hover_valid_ = false;
  emit cursorGeoChanged(qQNaN(), qQNaN());
  update();
  QWidget::leaveEvent(event);
}

void MapViewportWidget::keyPressEvent(QKeyEvent* event) {
  if (event == nullptr) {
    return;
  }
  switch (event->key()) {
    case Qt::Key_Escape:
      geo_map_.setMeasuring(false);
      geo_map_.clearMeasure();
      geo_map_.clearSelection();
      config_.edit_tool = MapEditTool::kPan;
      drag_plan_kind_ = -1;
      drag_plan_index_ = -1;
      selected_plan_kind_ = -1;
      selected_plan_index_ = -1;
      syncToolbar();
      syncCursor();
      publishMapStatus();
      publishSelection();
      emit planChanged();
      update();
      event->accept();
      return;
    case Qt::Key_Backspace:
    case Qt::Key_Delete:
      if (selected_plan_kind_ >= 0) {
        removePlanVertex(selected_plan_kind_, selected_plan_index_);
        event->accept();
        return;
      }
      if (undoLastPlanPoint()) {
        event->accept();
        return;
      }
      break;
    case Qt::Key_1:
      config_.edit_tool = MapEditTool::kPan;
      geo_map_.setMeasuring(false);
      syncToolbar();
      syncCursor();
      emit planChanged();
      event->accept();
      return;
    case Qt::Key_2:
      config_.edit_tool = MapEditTool::kWaypoint;
      geo_map_.setMeasuring(false);
      syncToolbar();
      syncCursor();
      emit planChanged();
      event->accept();
      return;
    case Qt::Key_3:
      config_.edit_tool = MapEditTool::kGeofence;
      geo_map_.setMeasuring(false);
      syncToolbar();
      syncCursor();
      emit planChanged();
      event->accept();
      return;
    case Qt::Key_4:
      config_.edit_tool = MapEditTool::kRally;
      geo_map_.setMeasuring(false);
      syncToolbar();
      syncCursor();
      emit planChanged();
      event->accept();
      return;
    case Qt::Key_M:
      geo_map_.setMeasuring(!geo_map_.measuring());
      if (geo_map_.measuring()) {
        config_.edit_tool = MapEditTool::kPan;
      }
      syncToolbar();
      syncCursor();
      publishMapStatus();
      emit planChanged();
      update();
      event->accept();
      return;
    case Qt::Key_F:
      recenter();
      event->accept();
      return;
    case Qt::Key_Plus:
    case Qt::Key_Equal: {
      const QPointF cursor(width() * 0.5, height() * 0.5);
      const QPointF anchor = screenToLatLon(cursor);
      config_.zoom = std::clamp(config_.zoom + 1.0, static_cast<double>(kMinZoom),
                                static_cast<double>(kMaxZoom));
      alignLatLonToScreen(anchor.x(), anchor.y(), cursor);
      user_moved_view_ = true;
      requestVisibleTiles();
      emit viewChanged(config_.center_latitude, config_.center_longitude,
                       config_.zoom);
      update();
      event->accept();
      return;
    }
    case Qt::Key_Minus: {
      const QPointF cursor(width() * 0.5, height() * 0.5);
      const QPointF anchor = screenToLatLon(cursor);
      config_.zoom = std::clamp(config_.zoom - 1.0, static_cast<double>(kMinZoom),
                                static_cast<double>(kMaxZoom));
      alignLatLonToScreen(anchor.x(), anchor.y(), cursor);
      user_moved_view_ = true;
      requestVisibleTiles();
      emit viewChanged(config_.center_latitude, config_.center_longitude,
                       config_.zoom);
      update();
      event->accept();
      return;
    }
    default:
      break;
  }
  QWidget::keyPressEvent(event);
}

void MapViewportWidget::mouseDoubleClickEvent(QMouseEvent* event) {
  if (event == nullptr || event->button() != Qt::LeftButton) {
    return;
  }
  if (panning_) {
    releaseMouse();
  }
  panning_ = false;
  if (hold_timer_ != nullptr) {
    hold_timer_->stop();
  }
  const QPointF cursor = event->position();
  const QPointF anchor = screenToLatLon(cursor);
  config_.zoom = std::clamp(config_.zoom + 1.0, static_cast<double>(kMinZoom),
                            static_cast<double>(kMaxZoom));
  alignLatLonToScreen(anchor.x(), anchor.y(), cursor);
  user_moved_view_ = true;
  requestVisibleTiles();
  emit viewChanged(config_.center_latitude, config_.center_longitude, config_.zoom);
  update();
  event->accept();
}

void MapViewportWidget::positionToolbar() {
  if (toolbar_ == nullptr) {
    return;
  }
  toolbar_->adjustSize();
  toolbar_->move(width() - toolbar_->width() - 10,
                 qMax(10, (height() - toolbar_->height()) / 2));
  toolbar_->raise();
}

void MapViewportWidget::syncToolbar() {
  if (toolbar_ == nullptr) {
    return;
  }
  toolbar_->setEditTool(config_.edit_tool);
  toolbar_->setMeasureChecked(geo_map_.measuring());
  toolbar_->setRecenterToolTip(config_.follow_channel.isEmpty()
                                   ? tr("Fit map to layers")
                                   : tr("Center on follow target"));
}

void MapViewportWidget::syncCursor() {
  if (panning_) {
    setCursor(Qt::ClosedHandCursor);
    return;
  }
  if (geo_map_.measuring()) {
    setCursor(Qt::CrossCursor);
    return;
  }
  switch (config_.edit_tool) {
    case MapEditTool::kWaypoint:
    case MapEditTool::kGeofence:
    case MapEditTool::kRally:
      setCursor(Qt::CrossCursor);
      break;
    case MapEditTool::kPan:
      setCursor(Qt::OpenHandCursor);
      break;
  }
}

void MapViewportWidget::recenter() { centerOnContent(); }

void MapViewportWidget::centerOnContent() {
  if (follow_target_.valid && !config_.follow_channel.isEmpty()) {
    user_moved_view_ = false;
    config_.center_latitude = follow_target_.latitude;
    config_.center_longitude = follow_target_.longitude;
    requestVisibleTiles();
    emit viewChanged(config_.center_latitude, config_.center_longitude, config_.zoom);
    update();
    return;
  }

  QVector<MapRenderLayerSnapshot> fit_layers = layers_;
  fit_layers += geo_map_.renderLayers();
  fitToLayers(fit_layers);
}

void MapViewportWidget::fitToLayers(const QVector<MapRenderLayerSnapshot>& fit_layers) {
  bool have_point = false;
  double min_lat = 0.0;
  double max_lat = 0.0;
  double min_lon = 0.0;
  double max_lon = 0.0;
  const auto include = [&](double latitude, double longitude) {
    if (!have_point) {
      min_lat = max_lat = latitude;
      min_lon = max_lon = longitude;
      have_point = true;
      return;
    }
    min_lat = std::min(min_lat, latitude);
    max_lat = std::max(max_lat, latitude);
    min_lon = std::min(min_lon, longitude);
    max_lon = std::max(max_lon, longitude);
  };
  for (const MapRenderLayerSnapshot& layer : fit_layers) {
    for (const MapGeoPoint& point : layer.points) {
      include(point.latitude, point.longitude);
    }
    for (const MapGeoLine& line : layer.lines) {
      for (const QPointF& lat_lon : line.lat_lon_points) {
        include(lat_lon.y(), lat_lon.x());
      }
    }
    for (const MapGeoPolygon& polygon : layer.polygons) {
      for (const QPointF& lat_lon : polygon.outer_ring) {
        include(lat_lon.y(), lat_lon.x());
      }
    }
  }
  if (!have_point || width() < 2 || height() < 2) {
    return;
  }

  config_.center_latitude = (min_lat + max_lat) * 0.5;
  config_.center_longitude = (min_lon + max_lon) * 0.5;
  const double pad = 48.0;
  for (int zoom = kMaxZoom; zoom >= kMinZoom; --zoom) {
    const QPointF north_west =
        MapProjection::LatLonToWorldPixel(max_lat, min_lon, zoom);
    const QPointF south_east =
        MapProjection::LatLonToWorldPixel(min_lat, max_lon, zoom);
    const double span_x = std::abs(south_east.x() - north_west.x());
    const double span_y = std::abs(south_east.y() - north_west.y());
    if (span_x <= width() - pad && span_y <= height() - pad) {
      config_.zoom = zoom;
      break;
    }
    config_.zoom = kMinZoom;
  }
  user_moved_view_ = true;
  requestVisibleTiles();
  emit viewChanged(config_.center_latitude, config_.center_longitude, config_.zoom);
  update();
}

void MapViewportWidget::drawCompass(QPainter* painter) const {
  const glass::OverlayTokens g = glass::Overlay();
  const QPointF center(34.0, 34.0);
  constexpr double kRadius = 15.0;
  painter->save();
  painter->setRenderHint(QPainter::Antialiasing, true);
  QPainterPath card;
  card.addEllipse(center, kRadius + 6.0, kRadius + 6.0);
  painter->fillPath(card, g.fill_deep);
  painter->setPen(QPen(QColor(g.rim.red(), g.rim.green(), g.rim.blue(), 70), 1.0));
  painter->drawPath(card);
  painter->setPen(QPen(QColor(g.accent.red(), g.accent.green(), g.accent.blue(), 200),
                       1.5));
  painter->setBrush(Qt::NoBrush);
  painter->drawEllipse(center, kRadius, kRadius);
  painter->setPen(QPen(QColor(248, 113, 113), 2.0));
  painter->drawLine(center, QPointF(center.x(), center.y() - kRadius + 3.0));
  QFont font = painter->font();
  font.setPixelSize(10);
  font.setBold(true);
  painter->setFont(font);
  painter->setPen(QColor(g.value.red(), g.value.green(), g.value.blue(), 230));
  painter->drawText(QRectF(center.x() - 8.0, center.y() - kRadius - 4.0, 16.0, 12.0),
                    Qt::AlignCenter, QStringLiteral("N"));
  painter->restore();
}

void MapViewportWidget::drawAttributeCard(QPainter* painter) const {
  if (painter == nullptr) {
    return;
  }
  const MapSelectionInfo info = currentSelection();
  if (!info.valid) {
    return;
  }
  QStringList rows;
  rows << QString::number(info.latitude, 'f', 6) + QStringLiteral(", ") +
              QString::number(info.longitude, 'f', 6);
  constexpr int kMaxRows = 6;
  const int shown = std::min(static_cast<int>(info.attributes.size()), kMaxRows);
  for (int i = 0; i < shown; ++i) {
    rows << info.attributes.at(i).key + QStringLiteral("   ") +
                info.attributes.at(i).value;
  }
  if (info.attributes.size() > shown) {
    rows << tr("+%1").arg(info.attributes.size() - shown);
  }

  const glass::OverlayTokens g = glass::Overlay();
  painter->save();
  painter->setRenderHint(QPainter::Antialiasing, true);
  QFont title_font = painter->font();
  title_font.setFamilies({QStringLiteral("PingFang SC"),
                          QStringLiteral(".AppleSystemUIFont"),
                          QStringLiteral("Noto Sans CJK SC"),
                          QStringLiteral("sans-serif")});
  title_font.setPixelSize(12);
  title_font.setBold(true);
  QFont body_font = title_font;
  body_font.setPixelSize(11);
  body_font.setBold(false);
  const QFontMetrics title_metrics(title_font);
  const QFontMetrics body_metrics(body_font);
  const QString subtitle = info.layer.isEmpty()
                               ? info.kind
                               : info.layer + QStringLiteral(" · ") + info.kind;
  const int text_limit = 260;
  int text_w = title_metrics.horizontalAdvance(info.title);
  text_w = std::max(text_w, body_metrics.horizontalAdvance(subtitle));
  for (const QString& row : rows) {
    text_w = std::max(text_w, body_metrics.horizontalAdvance(row));
  }
  text_w = std::min(text_w, text_limit);
  constexpr int kPad = 10;
  const int line_h = body_metrics.height() + 3;
  const int card_w = text_w + kPad * 2;
  const int card_h = kPad + title_metrics.height() + line_h +
                     static_cast<int>(rows.size()) * line_h + kPad;

  const QPointF anchor = latLonToScreen(info.latitude, info.longitude);
  const double toolbar_gap =
      toolbar_ != nullptr && toolbar_->isVisible() ? toolbar_->width() + 18.0 : 8.0;
  const double right_limit = std::max(8.0, width() - card_w - toolbar_gap);
  double left = anchor.x() + 16.0;
  if (left > right_limit) {
    left = anchor.x() - 16.0 - card_w;
  }
  left = std::clamp(left, 8.0, right_limit);
  double top = anchor.y() - card_h * 0.35;
  top = std::clamp(top, 8.0, std::max(8.0, height() - card_h - 8.0));
  const QRectF card(left, top, card_w, card_h);

  if (rect().adjusted(-40, -40, 40, 40).contains(anchor.toPoint())) {
    const QPointF edge(left < anchor.x() ? card.right() : card.left(),
                       std::clamp(anchor.y(), card.top() + 8.0, card.bottom() - 8.0));
    painter->setPen(QPen(QColor(g.accent.red(), g.accent.green(), g.accent.blue(), 180),
                         1.25));
    painter->drawLine(anchor, edge);
  }

  QPainterPath path;
  path.addRoundedRect(card, 10.0, 10.0);
  painter->fillPath(path, g.fill_deep);
  painter->setPen(QPen(QColor(g.rim.red(), g.rim.green(), g.rim.blue(), 80), 1.0));
  painter->drawPath(path);

  const QRectF text_rect(card.left() + kPad, card.top() + kPad, text_w,
                         title_metrics.height());
  painter->setFont(title_font);
  painter->setPen(QColor(g.value.red(), g.value.green(), g.value.blue(), 240));
  painter->drawText(text_rect, Qt::AlignLeft | Qt::AlignVCenter,
                    title_metrics.elidedText(info.title, Qt::ElideRight, text_w));
  painter->setFont(body_font);
  painter->setPen(QColor(g.accent.red(), g.accent.green(), g.accent.blue(), 220));
  const QRectF subtitle_rect(text_rect.left(), text_rect.bottom() + 1, text_w, line_h);
  painter->drawText(subtitle_rect, Qt::AlignLeft | Qt::AlignVCenter,
                    body_metrics.elidedText(subtitle, Qt::ElideRight, text_w));
  painter->setPen(QColor(g.unit.red(), g.unit.green(), g.unit.blue(), 230));
  double y = subtitle_rect.bottom();
  for (const QString& row : rows) {
    painter->drawText(QRectF(text_rect.left(), y, text_w, line_h),
                      Qt::AlignLeft | Qt::AlignVCenter,
                      body_metrics.elidedText(row, Qt::ElideMiddle, text_w));
    y += line_h;
  }
  painter->restore();
}

void MapViewportWidget::drawAttribution(QPainter* painter) const {
  const QString text = BaseLayerAttribution(config_.base_layer);
  if (text.isEmpty()) {
    return;
  }
  const glass::OverlayTokens g = glass::Overlay();
  painter->save();
  painter->setRenderHint(QPainter::Antialiasing, true);
  QFont font = painter->font();
  font.setPixelSize(10);
  painter->setFont(font);
  const QFontMetrics metrics(font);
  const QRectF card(width() - metrics.horizontalAdvance(text) - 22.0,
                    height() - metrics.height() - 16.0,
                    metrics.horizontalAdvance(text) + 14.0, metrics.height() + 8.0);
  QPainterPath path;
  path.addRoundedRect(card, 8.0, 8.0);
  painter->fillPath(path, g.fill_deep);
  painter->setPen(QPen(QColor(g.rim.red(), g.rim.green(), g.rim.blue(), 60), 1.0));
  painter->drawPath(path);
  painter->setPen(QColor(g.unit.red(), g.unit.green(), g.unit.blue(), 200));
  painter->drawText(card, Qt::AlignCenter, text);
  painter->restore();
}

void MapViewportWidget::drawVehicle(QPainter* painter) const {
  if (!follow_target_.valid || config_.follow_channel.isEmpty()) {
    return;
  }
  const QPointF screen =
      latLonToScreen(follow_target_.latitude, follow_target_.longitude);
  const double heading = std::isfinite(follow_target_.heading_deg)
                             ? follow_target_.heading_deg
                             : 0.0;
  painter->save();
  painter->setRenderHint(QPainter::Antialiasing, true);
  painter->setPen(QPen(QColor(120, 53, 15), 1.0));
  painter->setBrush(QColor(245, 158, 11));
  DrawArrow(painter, screen, 14.0, heading);
  painter->restore();
}

void MapViewportWidget::drawGcs(QPainter* painter) const {
  if (!gcs_target_.valid || config_.gcs_channel.isEmpty()) {
    return;
  }
  const QPointF screen =
      latLonToScreen(gcs_target_.latitude, gcs_target_.longitude);
  painter->save();
  painter->setRenderHint(QPainter::Antialiasing, true);
  painter->setPen(QPen(QColor(20, 83, 45), 1.5));
  painter->setBrush(QColor(34, 197, 94));
  painter->drawEllipse(screen, 7.0, 7.0);
  painter->setPen(QPen(QColor(255, 255, 255), 1.5));
  DrawCross(painter, screen, 4.0);
  painter->restore();
}

void MapViewportWidget::drawPlan(QPainter* painter) const {
  if (painter == nullptr) {
    return;
  }
  const glass::OverlayTokens g = glass::Overlay();
  const QColor route(g.accent.red(), g.accent.green(), g.accent.blue(), 230);
  const QColor fence(251, 146, 60, 220);
  painter->save();
  painter->setRenderHint(QPainter::Antialiasing, true);
  if (config_.geofence.size() >= 2) {
    QPainterPath fence_path;
    fence_path.moveTo(latLonToScreen(config_.geofence.front().latitude,
                                     config_.geofence.front().longitude));
    for (int i = 1; i < config_.geofence.size(); ++i) {
      fence_path.lineTo(latLonToScreen(config_.geofence.at(i).latitude,
                                       config_.geofence.at(i).longitude));
    }
    fence_path.closeSubpath();
    painter->setPen(QPen(fence, 2.0));
    painter->setBrush(QColor(251, 146, 60, 36));
    painter->drawPath(fence_path);
  }
  if (config_.waypoints.size() >= 2) {
    painter->setPen(QPen(route, 2.2));
    painter->setBrush(Qt::NoBrush);
    for (int i = 1; i < config_.waypoints.size(); ++i) {
      const QPointF from = latLonToScreen(config_.waypoints.at(i - 1).latitude,
                                          config_.waypoints.at(i - 1).longitude);
      const QPointF to = latLonToScreen(config_.waypoints.at(i).latitude,
                                        config_.waypoints.at(i).longitude);
      painter->drawLine(from, to);
      drawMissionArrow(painter, from, to, route);
    }
  }
  QFont font = painter->font();
  font.setPixelSize(10);
  font.setBold(true);
  painter->setFont(font);
  auto draw_selected = [&](const QPointF& screen) {
    painter->setPen(QPen(QColor(255, 255, 255, 230), 2.0));
    painter->setBrush(Qt::NoBrush);
    painter->drawEllipse(screen, 12.0, 12.0);
  };
  for (int i = 0; i < config_.waypoints.size(); ++i) {
    const QPointF screen = latLonToScreen(config_.waypoints.at(i).latitude,
                                          config_.waypoints.at(i).longitude);
    painter->setPen(QPen(QColor(g.fill.red(), g.fill.green(), g.fill.blue(), 220), 1.5));
    painter->setBrush(route);
    painter->drawEllipse(screen, 7.0, 7.0);
    if (selected_plan_kind_ == 0 && selected_plan_index_ == i) {
      draw_selected(screen);
    }
    painter->setPen(QColor(g.value.red(), g.value.green(), g.value.blue(), 240));
    painter->drawText(QRectF(screen.x() - 8.0, screen.y() - 14.0, 16.0, 12.0),
                      Qt::AlignCenter, QString::number(i + 1));
  }
  for (int i = 0; i < config_.geofence.size(); ++i) {
    const QPointF screen = latLonToScreen(config_.geofence.at(i).latitude,
                                          config_.geofence.at(i).longitude);
    painter->setPen(QPen(fence, 1.5));
    painter->setBrush(QColor(251, 146, 60, 220));
    painter->drawRect(QRectF(screen.x() - 4.0, screen.y() - 4.0, 8.0, 8.0));
    if (selected_plan_kind_ == 1 && selected_plan_index_ == i) {
      draw_selected(screen);
    }
  }
  for (int i = 0; i < config_.rally_points.size(); ++i) {
    const MapPlanPoint& rally = config_.rally_points.at(i);
    const QPointF screen = latLonToScreen(rally.latitude, rally.longitude);
    painter->setPen(QPen(QColor(52, 211, 153), 2.0));
    painter->setBrush(QColor(52, 211, 153, 180));
    painter->drawEllipse(screen, 6.0, 6.0);
    painter->setBrush(Qt::NoBrush);
    painter->drawEllipse(screen, 11.0, 11.0);
    if (selected_plan_kind_ == 2 && selected_plan_index_ == i) {
      draw_selected(screen);
    }
  }
  painter->restore();
}

void MapViewportWidget::drawMissionArrow(QPainter* painter, const QPointF& from,
                                        const QPointF& to,
                                        const QColor& color) const {
  const QPointF delta = to - from;
  const double length = std::hypot(delta.x(), delta.y());
  if (length < 8.0) {
    return;
  }
  const QPointF direction = delta / length;
  const QPointF normal(-direction.y(), direction.x());
  constexpr double kHead = 8.0;
  QPolygonF head;
  head << to
       << (to - direction * kHead + normal * (kHead * 0.45))
       << (to - direction * kHead - normal * (kHead * 0.45));
  painter->save();
  painter->setPen(Qt::NoPen);
  painter->setBrush(color.isValid() ? color : QColor(37, 99, 235));
  painter->drawPolygon(head);
  painter->restore();
}

void MapViewportWidget::showMapMenu(const QPoint& screen_pos) {
  const QPointF lat_lon = screenToLatLon(screen_pos);
  QMenu menu(this);
  QAction* copy_action = menu.addAction(tr("Copy coordinates"));
  QAction* center_action = menu.addAction(tr("Center here"));
  QAction* open_action = menu.addAction(tr("Open GeoJSON…"));
  QAction* measure_action =
      menu.addAction(geo_map_.measuring() ? tr("Stop measure") : tr("Measure"));
  QAction* clear_measure = nullptr;
  if (!geo_map_.measurePoints().isEmpty()) {
    clear_measure = menu.addAction(tr("Clear measure"));
  }
  QAction* undo_plan = menu.addAction(tr("Undo plan point"));
  QAction* clear_plan = menu.addAction(tr("Clear plan"));
  const GeoMapHit under = geo_map_.hitTest(
      screen_pos, layers_, [this](double latitude, double longitude) {
        return latLonToScreen(latitude, longitude);
      });
  QAction* hide_action = nullptr;
  QAction* remove_action = nullptr;
  if (under.valid) {
    hide_action = menu.addAction(tr("Hide layer"));
    if (under.layer_id.startsWith(QStringLiteral("geo:"))) {
      remove_action = menu.addAction(tr("Remove GeoJSON layer"));
    }
  }
  QVector<QAction*> layer_actions;
  if (!geo_map_.layers().isEmpty()) {
    menu.addSeparator();
    for (const GeoMapLayer& layer : geo_map_.layers()) {
      QAction* action = menu.addAction(layer.source);
      action->setCheckable(true);
      action->setChecked(layer.visible);
      action->setData(layer.id);
      layer_actions.push_back(action);
    }
  }
  QAction* chosen = menu.exec(mapToGlobal(screen_pos));
  if (chosen == copy_action) {
    QGuiApplication::clipboard()->setText(
        tr("%1, %2").arg(lat_lon.x(), 0, 'f', 6).arg(lat_lon.y(), 0, 'f', 6));
    return;
  }
  if (chosen == center_action) {
    config_.center_latitude = lat_lon.x();
    config_.center_longitude = lat_lon.y();
    user_moved_view_ = true;
    requestVisibleTiles();
    emit viewChanged(config_.center_latitude, config_.center_longitude,
                     config_.zoom);
    update();
    return;
  }
  if (chosen == open_action) {
    const QString path = QFileDialog::getOpenFileName(
        this, tr("Open GeoJSON"), QString(),
        tr("GeoJSON (*.geojson *.json)"));
    if (!path.isEmpty()) {
      loadGeoJsonFile(path);
    }
    return;
  }
  if (chosen == measure_action) {
    geo_map_.setMeasuring(!geo_map_.measuring());
    publishMapStatus();
    update();
    return;
  }
  if (chosen != nullptr && chosen == clear_measure) {
    geo_map_.clearMeasure();
    publishMapStatus();
    update();
    return;
  }
  if (chosen == undo_plan) {
    undoLastPlanPoint();
    return;
  }
  if (chosen == clear_plan) {
    config_.waypoints.clear();
    config_.geofence.clear();
    config_.rally_points.clear();
    emit planChanged();
    update();
    return;
  }
  if (chosen != nullptr && chosen == hide_action && under.valid) {
    if (geo_map_.setLayerVisible(under.layer_id, false)) {
      publishMapStatus();
      emit geoJsonSourcesChanged();
      update();
    } else {
      emit hideTopicLayerRequested(under.layer_id);
    }
    return;
  }
  if (chosen != nullptr && chosen == remove_action && under.valid) {
    if (geo_map_.removeLayer(under.layer_id)) {
      publishMapStatus();
      emit geoJsonSourcesChanged();
      update();
    }
    return;
  }
  if (chosen != nullptr && layer_actions.contains(chosen)) {
    geo_map_.setLayerVisible(chosen->data().toString(), chosen->isChecked());
    publishMapStatus();
    emit geoJsonSourcesChanged();
    update();
  }
}

void MapViewportWidget::armPressAndHold(const QPoint& screen_pos) {
  hold_press_pos_ = screen_pos;
  if (hold_timer_ != nullptr) {
    hold_timer_->start();
  }
}

bool MapViewportWidget::event(QEvent* event) {
  if (event != nullptr && event->type() == QEvent::Gesture) {
    auto* gesture_event = static_cast<QGestureEvent*>(event);
    if (auto* pinch = static_cast<QPinchGesture*>(
            gesture_event->gesture(Qt::PinchGesture))) {
      if (pinch->state() == Qt::GestureStarted) {
        if (hold_timer_ != nullptr) {
          hold_timer_->stop();
        }
        if (panning_) {
          releaseMouse();
        }
        panning_ = false;
        pinch_screen_ = pinch->centerPoint();
        pinch_anchor_ = screenToLatLon(pinch_screen_);
        pinch_active_ = true;
        gesture_event->accept(pinch);
        return true;
      }
      if (pinch_active_ && (pinch->state() == Qt::GestureUpdated ||
                            pinch->state() == Qt::GestureFinished)) {
        const double factor = pinch->scaleFactor();
        if (factor > 0.0 && !qFuzzyCompare(factor, 1.0)) {
          config_.zoom = std::clamp(config_.zoom + std::log2(factor),
                                    static_cast<double>(kMinZoom),
                                    static_cast<double>(kMaxZoom));
          alignLatLonToScreen(pinch_anchor_.x(), pinch_anchor_.y(),
                              pinch_screen_);
          user_moved_view_ = true;
          requestVisibleTiles();
          emit viewChanged(config_.center_latitude, config_.center_longitude,
                           config_.zoom);
          update();
        }
        if (pinch->state() == Qt::GestureFinished ||
            pinch->state() == Qt::GestureCanceled) {
          pinch_active_ = false;
        }
        gesture_event->accept(pinch);
        return true;
      }
    }
  }
  return QWidget::event(event);
}

bool MapViewportWidget::loadGeoJsonFile(const QString& path) {
  QString error;
  const QString id = geo_map_.loadFile(path, &error);
  publishMapStatus();
  emit geoJsonSourcesChanged();
  if (!id.isEmpty()) {
    QVector<MapRenderLayerSnapshot> opened;
    for (const MapRenderLayerSnapshot& layer : geo_map_.renderLayers()) {
      if (layer.channel == id) {
        opened.push_back(layer);
      }
    }
    fitToLayers(opened);
  }
  update();
  return !id.isEmpty();
}

void MapViewportWidget::handleMapClick(const QPoint& screen_pos) {
  const QPointF lat_lon = screenToLatLon(screen_pos);
  if (geo_map_.measuring()) {
    geo_map_.addMeasurePoint(lat_lon.x(), lat_lon.y());
    publishMapStatus();
    update();
    return;
  }
  QVector<MapPlanPoint>* points = nullptr;
  switch (config_.edit_tool) {
    case MapEditTool::kWaypoint:
      points = &config_.waypoints;
      break;
    case MapEditTool::kGeofence:
      points = &config_.geofence;
      break;
    case MapEditTool::kRally:
      points = &config_.rally_points;
      break;
    case MapEditTool::kPan:
      break;
  }
  if (points != nullptr) {
    points->push_back(MapPlanPoint{lat_lon.x(), lat_lon.y()});
    selected_plan_kind_ = config_.edit_tool == MapEditTool::kGeofence
                              ? 1
                              : config_.edit_tool == MapEditTool::kRally ? 2 : 0;
    selected_plan_index_ = points->size() - 1;
    geo_map_.clearSelection();
    emit planChanged();
    publishSelection();
    update();
    return;
  }
  int plan_kind = -1;
  int plan_index = -1;
  if (hitPlanVertex(screen_pos, 10.0, &plan_kind, &plan_index)) {
    selected_plan_kind_ = plan_kind;
    selected_plan_index_ = plan_index;
    geo_map_.clearSelection();
    publishMapStatus();
    publishSelection();
    update();
    return;
  }
  selected_plan_kind_ = -1;
  selected_plan_index_ = -1;
  geo_map_.setSelection(geo_map_.hitTest(
      screen_pos, layers_, [this](double latitude, double longitude) {
        return latLonToScreen(latitude, longitude);
      }));
  publishMapStatus();
  publishSelection();
  update();
}

void MapViewportWidget::publishMapStatus() {
  emit mapStatusChanged(geo_map_.statusLine(config_.distance_unit));
}

QVector<MapPlanPoint>* MapViewportWidget::planPoints(int kind) {
  if (kind == 0) {
    return &config_.waypoints;
  }
  if (kind == 1) {
    return &config_.geofence;
  }
  if (kind == 2) {
    return &config_.rally_points;
  }
  return nullptr;
}

const QVector<MapPlanPoint>* MapViewportWidget::planPoints(int kind) const {
  return const_cast<MapViewportWidget*>(this)->planPoints(kind);
}

MapSelectionInfo MapViewportWidget::currentSelection() const {
  if (selected_plan_kind_ >= 0) {
    const QVector<MapPlanPoint>* points = planPoints(selected_plan_kind_);
    if (points != nullptr && selected_plan_index_ >= 0 &&
        selected_plan_index_ < points->size()) {
      const MapPlanPoint& point = points->at(selected_plan_index_);
      MapSelectionInfo info;
      info.valid = true;
      info.plan_kind = selected_plan_kind_;
      info.plan_index = selected_plan_index_;
      info.latitude = point.latitude;
      info.longitude = point.longitude;
      info.layer = tr("Plan");
      if (selected_plan_kind_ == 1) {
        info.kind = tr("Geofence");
        info.title = tr("Geofence %1").arg(selected_plan_index_ + 1);
      } else if (selected_plan_kind_ == 2) {
        info.kind = tr("Rally");
        info.title = tr("Rally %1").arg(selected_plan_index_ + 1);
      } else {
        info.kind = tr("Waypoint");
        info.title = tr("Waypoint %1").arg(selected_plan_index_ + 1);
      }
      info.attributes.push_back(
          {tr("Index"), QString::number(selected_plan_index_ + 1)});
      info.attributes.push_back({tr("Count"), QString::number(points->size())});
      return info;
    }
  }
  const GeoMapHit& hit = geo_map_.selection();
  if (!hit.valid) {
    return {};
  }
  MapSelectionInfo info;
  info.valid = true;
  info.title = hit.label;
  info.layer = hit.layer_name;
  info.latitude = hit.latitude;
  info.longitude = hit.longitude;
  info.attributes = hit.attributes;
  switch (hit.kind) {
    case GeoMapFeatureKind::kLine:
      info.kind = tr("Line");
      break;
    case GeoMapFeatureKind::kPolygon:
      info.kind = tr("Polygon");
      break;
    case GeoMapFeatureKind::kPoint:
      info.kind = tr("Point");
      break;
    case GeoMapFeatureKind::kNone:
      info.kind = tr("Feature");
      break;
  }
  if (info.title.isEmpty()) {
    info.title = info.kind;
  }
  return info;
}

void MapViewportWidget::publishSelection() {
  const MapSelectionInfo info = currentSelection();
  const auto same_number = [](double a, double b) {
    return (std::isfinite(a) && std::isfinite(b) && std::abs(a - b) < 1e-8) ||
           (!std::isfinite(a) && !std::isfinite(b));
  };
  bool same = info.valid == last_selection_.valid && info.title == last_selection_.title &&
              info.kind == last_selection_.kind && info.layer == last_selection_.layer &&
              info.plan_kind == last_selection_.plan_kind &&
              info.plan_index == last_selection_.plan_index &&
              same_number(info.latitude, last_selection_.latitude) &&
              same_number(info.longitude, last_selection_.longitude) &&
              info.attributes.size() == last_selection_.attributes.size();
  if (same) {
    for (int i = 0; i < info.attributes.size(); ++i) {
      if (info.attributes.at(i).key != last_selection_.attributes.at(i).key ||
          info.attributes.at(i).value != last_selection_.attributes.at(i).value) {
        same = false;
        break;
      }
    }
  }
  if (same) {
    return;
  }
  last_selection_ = info;
  emit selectionChanged(info);
}

void MapViewportWidget::setPlanVertex(int kind, int index, double latitude,
                                      double longitude) {
  QVector<MapPlanPoint>* points = planPoints(kind);
  if (points == nullptr || index < 0 || index >= points->size()) {
    return;
  }
  (*points)[index] = MapPlanPoint{latitude, longitude};
  selected_plan_kind_ = kind;
  selected_plan_index_ = index;
  geo_map_.clearSelection();
  emit planChanged();
  publishSelection();
  update();
}

void MapViewportWidget::removePlanVertex(int kind, int index) {
  QVector<MapPlanPoint>* points = planPoints(kind);
  if (points == nullptr || index < 0 || index >= points->size()) {
    return;
  }
  points->removeAt(index);
  if (selected_plan_kind_ == kind) {
    if (selected_plan_index_ == index || selected_plan_index_ >= points->size()) {
      selected_plan_kind_ = -1;
      selected_plan_index_ = -1;
    } else if (selected_plan_index_ > index) {
      --selected_plan_index_;
    }
  }
  emit planChanged();
  publishSelection();
  update();
}

bool MapViewportWidget::hitPlanVertex(const QPointF& screen, double tolerance_px,
                                      int* kind, int* index) const {
  if (kind == nullptr || index == nullptr) {
    return false;
  }
  double best = tolerance_px;
  int best_kind = -1;
  int best_index = -1;
  auto consider = [&](const QVector<MapPlanPoint>& points, int layer_kind) {
    for (int i = 0; i < points.size(); ++i) {
      const QPointF pos =
          latLonToScreen(points.at(i).latitude, points.at(i).longitude);
      const double dist = QLineF(screen, pos).length();
      if (dist <= best) {
        best = dist;
        best_kind = layer_kind;
        best_index = i;
      }
    }
  };
  consider(config_.waypoints, 0);
  consider(config_.geofence, 1);
  consider(config_.rally_points, 2);
  if (best_kind < 0) {
    return false;
  }
  *kind = best_kind;
  *index = best_index;
  return true;
}

bool MapViewportWidget::undoLastPlanPoint() {
  QVector<MapPlanPoint>* points = nullptr;
  switch (config_.edit_tool) {
    case MapEditTool::kWaypoint:
      points = &config_.waypoints;
      break;
    case MapEditTool::kGeofence:
      points = &config_.geofence;
      break;
    case MapEditTool::kRally:
      points = &config_.rally_points;
      break;
    case MapEditTool::kPan:
      if (!config_.waypoints.isEmpty()) {
        points = &config_.waypoints;
      } else if (!config_.geofence.isEmpty()) {
        points = &config_.geofence;
      } else if (!config_.rally_points.isEmpty()) {
        points = &config_.rally_points;
      }
      break;
  }
  if (points == nullptr || points->isEmpty()) {
    return false;
  }
  points->removeLast();
  if (selected_plan_index_ >= points->size()) {
    selected_plan_kind_ = -1;
    selected_plan_index_ = -1;
  }
  emit planChanged();
  publishSelection();
  update();
  return true;
}

void MapViewportWidget::drawHoverPreview(QPainter* painter) const {
  if (painter == nullptr || !hover_valid_ || panning_ || drag_plan_kind_ >= 0) {
    return;
  }
  const QPointF cursor = latLonToScreen(hover_lat_lon_.x(), hover_lat_lon_.y());
  const glass::OverlayTokens g = glass::Overlay();
  painter->save();
  painter->setRenderHint(QPainter::Antialiasing, true);

  if (geo_map_.measuring() && !geo_map_.measurePoints().isEmpty()) {
    const QPointF last = geo_map_.measurePoints().last();
    const QPointF from = latLonToScreen(last.x(), last.y());
    QPen pen(QColor(234, 179, 8, 180));
    pen.setWidthF(1.5);
    pen.setStyle(Qt::DashLine);
    painter->setPen(pen);
    painter->drawLine(from, cursor);
    painter->setBrush(QColor(234, 179, 8, 160));
    painter->drawEllipse(cursor, 3.5, 3.5);
    painter->restore();
    return;
  }

  const QVector<MapPlanPoint>* points = nullptr;
  QColor color(g.accent.red(), g.accent.green(), g.accent.blue(), 200);
  switch (config_.edit_tool) {
    case MapEditTool::kWaypoint:
      points = &config_.waypoints;
      break;
    case MapEditTool::kGeofence:
      points = &config_.geofence;
      color = QColor(251, 146, 60, 200);
      break;
    case MapEditTool::kRally:
      points = &config_.rally_points;
      color = QColor(52, 211, 153, 200);
      break;
    case MapEditTool::kPan:
      painter->restore();
      return;
  }
  painter->setPen(QPen(color, 1.5, Qt::DashLine));
  painter->setBrush(color);
  if (points != nullptr && !points->isEmpty()) {
    const QPointF from = latLonToScreen(points->last().latitude,
                                        points->last().longitude);
    painter->drawLine(from, cursor);
  }
  painter->drawEllipse(cursor, 4.0, 4.0);
  painter->restore();
}

void MapViewportWidget::drawMeasure(QPainter* painter) const {
  const QVector<QPointF>& points = geo_map_.measurePoints();
  if (points.isEmpty() || painter == nullptr) {
    return;
  }
  painter->save();
  painter->setRenderHint(QPainter::Antialiasing, true);
  QPen pen(QColor(234, 179, 8));
  pen.setWidth(2);
  pen.setStyle(Qt::DashLine);
  painter->setPen(pen);
  painter->setBrush(QColor(234, 179, 8));
  QPointF previous;
  for (int i = 0; i < points.size(); ++i) {
    const QPointF screen = latLonToScreen(points.at(i).x(), points.at(i).y());
    painter->drawEllipse(screen, 4.0, 4.0);
    if (i > 0) {
      painter->drawLine(previous, screen);
    }
    previous = screen;
  }
  if (points.size() == 2) {
    const QString label = geo_map_.formatMeasure(config_.distance_unit);
    const QPointF mid = (latLonToScreen(points.at(0).x(), points.at(0).y()) +
                         latLonToScreen(points.at(1).x(), points.at(1).y())) *
                        0.5;
    painter->setPen(QColor(15, 23, 42));
    painter->drawText(mid + QPointF(6.0, -6.0), label);
  }
  painter->restore();
}

}  // namespace map
}  // namespace autoviz
