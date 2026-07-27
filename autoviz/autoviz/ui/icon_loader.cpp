/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/icon_loader.hpp"

#include <QCursor>
#include <QImage>
#include <QPainter>
#include <QPixmap>
#include <QPixmapCache>
#include <QSvgRenderer>

#include "autoviz/common/display_status.hpp"

namespace autoviz {
namespace {

QString MapDisplayIcon(const QString& display_type) {
  return QStringLiteral(":/autoviz/icons/classes/") + display_type;
}

QString MapToolIconBase(const QString& tool_id) {
  static const struct {
    const char* id;
    const char* icon;
  } kMap[] = {
      {"Interact", "classes/Interact"},
      {"MoveCamera", "classes/MoveCamera"},
      {"Select", "classes/Select"},
      {"FocusCamera", "classes/FocusCamera"},
      {"Measure", "classes/Measure"},
      {"PoseEstimate", "classes/SetInitialPose"},
      {"NavGoal", "classes/SetGoal"},
      {"PublishPoint", "classes/PublishPoint"},
  };
  for (const auto& entry : kMap) {
    if (tool_id == QLatin1String(entry.id)) {
      return QStringLiteral(":/autoviz/icons/") +
             QLatin1String(entry.icon);
    }
  }
  return QStringLiteral(":/autoviz/icons/cursor");
}

QString MapPanelIcon(const QString& panel_id) {
  static const struct {
    const char* id;
    const char* icon;
  } kMap[] = {
      {"Displays", "classes/Displays.svg"},
      {"Views", "classes/Views.svg"},
      {"Help", "classes/Help.svg"},
      {"Selection", "crosshair.svg"},
      {"ToolProperties", "menu.svg"},
      {"Time", "rotate.svg"},
      {"Playback", "rotate_cam.svg"},
      {"TfTree", "classes/XYZ.svg"},
      {"Transformation", "classes/TF.png"},
      {"Image", "classes/RGB8.svg"},
      {"Channels", "classes/Displays.svg"},
  };
  for (const auto& entry : kMap) {
    if (panel_id == QLatin1String(entry.id)) {
      return QStringLiteral(":/autoviz/icons/") + entry.icon;
    }
  }
  return QStringLiteral(":/autoviz/icons/menu.svg");
}

bool IsVisiblePixmap(const QPixmap& pixmap) {
  if (pixmap.isNull() || pixmap.width() <= 0 || pixmap.height() <= 0) {
    return false;
  }
  const QImage image = pixmap.toImage().convertToFormat(QImage::Format_ARGB32);
  for (int y = 0; y < image.height(); ++y) {
    for (int x = 0; x < image.width(); ++x) {
      if (qAlpha(image.pixel(x, y)) > 16) {
        return true;
      }
    }
  }
  return false;
}

constexpr int kIconMasterSize = 256;
constexpr int kIconSizes[] = {16, 20, 24, 28, 32, 48, 64, 96, 128, 192, kIconMasterSize};

QIcon BuildIconFromMaster(const QPixmap& master) {
  QIcon icon;
  if (!IsVisiblePixmap(master)) {
    return icon;
  }
  for (const int size : kIconSizes) {
    if (master.width() == size && master.height() == size) {
      icon.addPixmap(master);
      continue;
    }
    icon.addPixmap(master.scaled(size, size, Qt::KeepAspectRatio,
                                 Qt::SmoothTransformation));
  }
  return icon;
}

QPixmap UpscalePixmap(const QPixmap& source) {
  if (source.isNull()) {
    return {};
  }
  if (source.width() >= kIconMasterSize && source.height() >= kIconMasterSize) {
    return source;
  }
  return source.scaled(kIconMasterSize, kIconMasterSize, Qt::KeepAspectRatio,
                       Qt::SmoothTransformation);
}

QPixmap RenderSvgPixmap(const QString& resource_path, int size) {
  QSvgRenderer renderer(resource_path);
  if (!renderer.isValid()) {
    return {};
  }
  QPixmap pixmap(size, size);
  pixmap.fill(Qt::transparent);
  QPainter painter(&pixmap);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.setRenderHint(QPainter::SmoothPixmapTransform, true);
  painter.setRenderHint(QPainter::TextAntialiasing, true);
  renderer.render(&painter, QRectF(0, 0, size, size));
  painter.end();
  return pixmap;
}

QIcon IconFromSvgResource(const QString& resource_path) {
  QSvgRenderer renderer(resource_path);
  if (!renderer.isValid()) {
    return {};
  }
  QIcon icon;
  for (const int size : kIconSizes) {
    QPixmap pixmap(size, size);
    pixmap.fill(Qt::transparent);
    QPainter painter(&pixmap);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setRenderHint(QPainter::SmoothPixmapTransform, true);
    painter.setRenderHint(QPainter::TextAntialiasing, true);
    renderer.render(&painter, QRectF(0, 0, size, size));
    painter.end();
    if (IsVisiblePixmap(pixmap)) {
      icon.addPixmap(pixmap);
    }
  }
  return icon;
}

constexpr int kStatusIconSizes[] = {12, 14, 16, 20, 24, 28, 32};

QIcon StatusIconFromSvgResource(const QString& resource_path) {
  QSvgRenderer renderer(resource_path);
  if (!renderer.isValid()) {
    return {};
  }
  QIcon icon;
  for (const int logical_size : kStatusIconSizes) {
    for (const qreal dpr : {1.0, 2.0, 3.0}) {
      const int physical_size = qRound(logical_size * dpr);
      QPixmap pixmap(physical_size, physical_size);
      pixmap.fill(Qt::transparent);
      pixmap.setDevicePixelRatio(dpr);
      QPainter painter(&pixmap);
      painter.setRenderHint(QPainter::Antialiasing, true);
      painter.setRenderHint(QPainter::SmoothPixmapTransform, true);
      painter.setRenderHint(QPainter::TextAntialiasing, true);
      renderer.render(&painter,
                      QRectF(0, 0, logical_size, logical_size));
      painter.end();
      if (IsVisiblePixmap(pixmap)) {
        icon.addPixmap(pixmap);
      }
    }
  }
  return icon;
}

QIcon IconFromPngResource(const QString& resource_path) {
  const QPixmap source(resource_path);
  if (source.isNull() || !IsVisiblePixmap(source)) {
    return {};
  }
  return BuildIconFromMaster(UpscalePixmap(source));
}

QIcon IconFromResource(const QString& resource_path) {
  if (resource_path.endsWith(QLatin1String(".svg"), Qt::CaseInsensitive)) {
    QIcon icon = IconFromSvgResource(resource_path);
    if (!icon.isNull()) {
      return icon;
    }
  }

  if (resource_path.endsWith(QLatin1String(".png"), Qt::CaseInsensitive)) {
    QIcon icon = IconFromPngResource(resource_path);
    if (!icon.isNull()) {
      return icon;
    }
  }

  return IconFromPngResource(resource_path);
}

QIcon LoadDisplayIcon(const QString& display_type) {
  const QString base = MapDisplayIcon(display_type);
  // Display icons: vector SVG only (never upscaled legacy PNG).
  QIcon icon = IconFromSvgResource(base + QStringLiteral(".svg"));
  if (!icon.isNull()) {
    return icon;
  }
  return IconFromSvgResource(
      QStringLiteral(":/autoviz/icons/default_class_icon.svg"));
}

QIcon LoadToolIcon(const QString& tool_id) {
  const QString base = MapToolIconBase(tool_id);
  QSvgRenderer renderer(base + QStringLiteral(".svg"));
  if (!renderer.isValid()) {
    return IconFromSvgResource(
        QStringLiteral(":/autoviz/icons/default_class_icon.svg"));
  }
  QIcon icon;
  constexpr int kToolIconSizes[] = {16, 20, 24, 28, 32, 40, 48, 64, 96, 128};
  for (const int logical_size : kToolIconSizes) {
    for (const qreal dpr : {1.0, 2.0, 3.0}) {
      const int physical_size = qRound(logical_size * dpr);
      QPixmap pixmap(physical_size, physical_size);
      pixmap.fill(Qt::transparent);
      pixmap.setDevicePixelRatio(dpr);
      QPainter painter(&pixmap);
      painter.setRenderHint(QPainter::Antialiasing, true);
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
      painter.setRenderHint(QPainter::SmoothPixmapTransform, true);
#else
      painter.setRenderHint(QPainter::HighQualityAntialiasing, true);
      painter.setRenderHint(QPainter::SmoothPixmapTransform, true);
#endif
      painter.setRenderHint(QPainter::TextAntialiasing, true);
      renderer.render(&painter,
                      QRectF(0, 0, logical_size, logical_size));
      painter.end();
      if (IsVisiblePixmap(pixmap)) {
        icon.addPixmap(pixmap);
      }
    }
  }
  if (!icon.isNull()) {
    return icon;
  }
  return IconFromSvgResource(
      QStringLiteral(":/autoviz/icons/default_class_icon.svg"));
}

}  // namespace

QIcon IconLoader::applicationIcon() {
  return load(QStringLiteral(":/autoviz/icons/aviz.svg"));
}

QIcon IconLoader::load(const QString& resource_path) {
  QIcon icon = IconFromResource(resource_path);
  if (!icon.isNull()) {
    return icon;
  }
  return IconFromSvgResource(QStringLiteral(":/autoviz/icons/default_class_icon.svg"));
}

QIcon IconLoader::displayIcon(const QString& display_type) {
  return LoadDisplayIcon(display_type);
}

QIcon IconLoader::toolIcon(const QString& tool_id) {
  return LoadToolIcon(tool_id);
}

QCursor IconLoader::defaultCursor() {
  return QCursor(Qt::ArrowCursor);
}

QCursor IconLoader::makeIconCursor(const QPixmap& icon,
                                   const QString& cache_key) {
  QPixmap cursor_img;
  if (QPixmapCache::find(cache_key, &cursor_img)) {
    return QCursor(cursor_img, 1, 1);
  }

  constexpr int kCursorSize = 32;
  QPixmap base_cursor =
      RenderSvgPixmap(QStringLiteral(":/autoviz/icons/cursor.svg"), kCursorSize);
  if (base_cursor.isNull()) {
    base_cursor = load(QStringLiteral(":/autoviz/icons/cursor.svg"))
                      .pixmap(kCursorSize, kCursorSize);
  }
  if (base_cursor.isNull() || icon.isNull()) {
    return defaultCursor();
  }

  cursor_img = QPixmap(kCursorSize, kCursorSize);
  cursor_img.fill(Qt::transparent);

  int draw_x = 12;
  int draw_y = 16;
  if (draw_x + icon.width() > kCursorSize) {
    draw_x = kCursorSize - icon.width();
  }
  if (draw_y + icon.height() > kCursorSize) {
    draw_y = kCursorSize - icon.height();
  }

  QPainter painter(&cursor_img);
  painter.drawPixmap(0, 0, base_cursor);
  painter.drawPixmap(draw_x, draw_y, icon);
  painter.end();

  QPixmapCache::insert(cache_key, cursor_img);
  return QCursor(cursor_img, 1, 1);
}

QCursor IconLoader::toolCursor(const QString& tool_id) {
  if (tool_id == QLatin1String("MoveCamera")) {
    return defaultCursor();
  }
  if (tool_id == QLatin1String("Interact")) {
    return QCursor(Qt::PointingHandCursor);
  }

  const QIcon icon = toolIcon(tool_id);
  QPixmap pixmap = icon.pixmap(22, 22);
  if (pixmap.isNull()) {
    pixmap = icon.pixmap(16, 16);
  }
  if (pixmap.isNull()) {
    return defaultCursor();
  }
  return makeIconCursor(pixmap, QStringLiteral("tool_cursor:") + tool_id);
}

QIcon IconLoader::panelIcon(const QString& panel_id) {
  return load(MapPanelIcon(panel_id));
}

QIcon IconLoader::statusIcon(display::DisplayStatusLevel level, bool enabled) {
  if (!enabled) {
    return StatusIconFromSvgResource(
        QStringLiteral(":/autoviz/icons/status_disabled.svg"));
  }
  switch (level) {
    case display::DisplayStatusLevel::kError:
      return StatusIconFromSvgResource(
          QStringLiteral(":/autoviz/icons/status_error.svg"));
    case display::DisplayStatusLevel::kWarn:
      return StatusIconFromSvgResource(
          QStringLiteral(":/autoviz/icons/status_warn.svg"));
    case display::DisplayStatusLevel::kOk:
    default:
      return StatusIconFromSvgResource(
          QStringLiteral(":/autoviz/icons/status_ok.svg"));
  }
}

}  // namespace autoviz
